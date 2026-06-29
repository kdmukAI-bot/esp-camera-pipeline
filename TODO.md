# TODO

## CI / GitHub Actions
- **No GitHub Actions CI in this repo yet.** Add a workflow to catch regressions:
  - a build check (compile this component against a sample app / minimal harness),
  - optionally a `clang-format` gate (the `k_quirc` submodule already has one —
    see its `.github/workflows/validate.yml`).
