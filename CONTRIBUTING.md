# Contributing

```
git clone https://github.com/tahakocp/urdflint
cd urdflint
cargo test
```

Adding a rule:

1. Add a `Rule` to `src/rules.rs` with a one-line summary and a short "why".
2. Implement it in `src/lint.rs`. Checks take parsed nodes and push a `Diag`; they never print.
3. Drop a minimal file named `<CODE>_<what>.urdf` into `tests/fixtures/bad/`. It must trigger that code and nothing else.
4. If valid models could look similar, add one to `tests/fixtures/good/`.

A rule is welcome when the mistake is common and the check is almost never wrong. If a legitimate robot trips it, make it a warning or skip the case. Warnings can be silenced with `--allow`; errors cannot, so be stricter about those.

No new dependencies without a reason. The binary is under a megabyte and builds in a few seconds, which is most of its appeal in CI.
