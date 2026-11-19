# urdflint

A single-binary linter for URDF robot descriptions. It finds broken kinematic trees, impossible inertias, empty joint ranges and missing meshes before your simulator does, and it reports the line number.

```
$ urdflint examples/broken_arm.urdf
examples/broken_arm.urdf:2: error[E015]: 2 root links: `base_link`, `wrist`
examples/broken_arm.urdf:13: error[E032]: link `shoulder` principal moments 4.0000e-4 + 4.0000e-4 < 9.0000e-2
examples/broken_arm.urdf:15: error[E040]: `meshes/shoulder.stl` does not exist (examples/meshes/shoulder.stl)
examples/broken_arm.urdf:17: warning[W032]: link `forearm` is moved by a joint but has no <inertial>
examples/broken_arm.urdf:20: error[E030]: link `wrist` mass is 0
examples/broken_arm.urdf:29: warning[W022]: joint `shoulder_pan` range [-170, 170] is wider than two turns each way; degrees?
examples/broken_arm.urdf:35: error[E022]: joint `elbow` lower 2 > upper 0
examples/broken_arm.urdf:37: error[E012]: joint `wrist_roll` child link `wrist_link` is not defined
examples/broken_arm.urdf: 4 links, 3 joints (3 DOF), 6.200 kg; 6 error(s), 2 warning(s)
```

Running it over a public collection of 320 URDF files (see [numbers](#numbers)) took under a second and flagged 81 of them with at least one error.

## Why

Most URDF mistakes show up late. A typo in a `<child link>` loads fine in some tools and drops a limb in others. An inertia matrix that no real body could have makes a simulator explode on the first step, with a message that mentions neither the link nor the file. Mesh exports in millimetres produce a robot the size of a building. With robot descriptions now being exported by CAD plugins, hand review does not scale and a quick check in CI does.

`urdflint` checks what can be checked statically: structure, numbers, and files on disk. It does not simulate anything.

## Install

Prebuilt binaries for Linux, macOS (Apple silicon) and Windows are attached to each [release](https://github.com/TahaKocpinar/urdflint/releases). With a Rust toolchain:

```
cargo install --git https://github.com/TahaKocpinar/urdflint
```

## Use

```
urdflint robot.urdf
urdflint description/            # every *.urdf below the directory
urdflint --package-path my_robot_description=./src/my_robot_description robot.urdf
urdflint --format github --deny-warnings description/
urdflint --explain E032
```

Exit status is 0 when there are no errors, 1 when there are (or warnings, with `--deny-warnings`), 2 for a usage or I/O problem. `--format json` prints an array of `{file, line, code, severity, message}`. `--allow W032` silences a rule; errors can be allowed too, but think first.

In GitHub Actions, `--format github` turns findings into inline annotations on the pull request:

```yaml
- run: |
    curl -sL https://github.com/TahaKocpinar/urdflint/releases/download/v0.1.0/urdflint-v0.1.0-x86_64-unknown-linux-gnu.tar.gz | tar xz --strip-components=1
    ./urdflint --format github --deny-warnings description/
```

### Mesh paths

Relative paths are resolved against the URDF's directory. `package://name/...` is looked up in this order: `--package-path name=DIR`, a directory called `name` next to the URDF or any of its eight nearest ancestors, then each entry of `ROS_PACKAGE_PATH`. If none matches, the mesh is skipped with `W041` rather than guessed. `--no-meshes` skips file checks altogether.

## Rules

31 rules; `urdflint --list-rules` prints them and `--explain CODE` gives the reasoning. The ones that catch the most in practice:

| Code | What it catches |
|---|---|
| E012 | joint points at a link that does not exist |
| E013, E014, E015 | two parents, a loop, or more than one root |
| E022, E023 | lower limit above upper limit, zero-length joint axis |
| E030, E031, E032 | zero mass, matrix not positive definite, principal moments that break `A + B >= C` |
| E040 | mesh file missing on disk |
| W022 | revolute limits that look like degrees |
| W030, W033 | masses or moment ratios that look like a unit slip |
| W040 | STL whose size after scale is over 20 m or under 1 mm |

The inertia test diagonalises the 3x3 matrix (Jacobi rotations) instead of checking only the diagonal, so off-diagonal terms that make the matrix indefinite are caught. The triangle inequality uses a relative tolerance of 1e-6 because flat plates exported from CAD sit exactly on the boundary.

## Numbers

Corpus: [Daniella1/urdf_files_dataset](https://github.com/Daniella1/urdf_files_dataset) at commit `81f4cda`, 320 `.urdf` files from ROS, Drake and MATLAB robot descriptions. Reproduce with `scripts/corpus.sh`. Release build, files already in the page cache: 0.8 s wall for everything, including reading the STL meshes that are present.

| Finding | Files |
|---|---|
| principal moments break the triangle inequality (E032) | 28 |
| mesh missing on disk (E040) | 23 |
| sphere, box or cylinder with zero or negative size (E041) | 15 |
| xacro left unexpanded (E003) | 11 |
| matrix not positive definite (E031) | 8 |
| joint refers to an undefined link (E012) | 5 |
| zero or negative mass (E030) | 4 |
| moving link with no inertial (W032) | 106 |
| zero effort or velocity limit (W021) | 97 |

81 of the 320 files have at least one error; 66 still do when the mesh-path rule E040 is excluded. Not all of these will hurt you. A link with no inertial is fine in a visual-only model, and a zero effort limit is fine if nothing ever applies torque. Those are warnings for that reason, and `--allow` exists for them. E040 depends on how packages are laid out in that dataset, so read that line as an upper bound.

A zero-length axis (E023) did not show up at all, because every case in this corpus sits on a fixed joint, where the axis is ignored and is not checked.

## What it does not do

- No xacro. Run `xacro` first and lint its output; unexpanded files get E003.
- Meshes are only opened for STL, and only to read the bounding box. OBJ and DAE are checked for existence.
- It does not check that inertia matches geometry, only that the numbers could belong to some body.
- SDF and MJCF are out of scope for now.
- No auto-fix.

## FAQ

**How is this different from `check_urdf`?**
`check_urdf` from urdfdom parses the file and prints the link tree, which settles the structural questions. As far as I know it does not look at whether inertias are physically possible, whether limits are ordered, or whether mesh files exist. Use both if you like; they overlap on E010 to E015.

**Doesn't my simulator already reject bad inertias?**
Some do, some do not, and the ones that do (MuJoCo, for instance) report it when you load the model, on the machine that has the simulator installed. This runs in CI in a second.

**Will it flag my valid robot?**
Possibly. The warnings in particular are heuristics. Open an issue with the smallest file that shows it; the `false_positive` template asks for exactly that.

**Why not Python?**
`pip install` on a robot or in a minimal CI image is slower and more fragile than copying a 650 KB file.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). A new rule is one table entry, one check, and one fixture file.

## License

MIT
