# Main branch protection

`main-protection.json` is the REST PUT branch-protection payload for
`cmu-impactlab/patrolbot` only. Checking it in does not apply GitHub settings.
After this bootstrap PR passes validation and is merged, the parent integrator
applies the merged file from the repository root:

```sh
gh api --method PUT repos/cmu-impactlab/patrolbot/branches/main/protection \
  --input .github/main-protection.json
gh api repos/cmu-impactlab/patrolbot/branches/main/protection
```

Read back protection after applying it and compare the effective settings with
the file: exactly `contract` and `native-test-image` in required status `checks`,
each bound to GitHub Actions App ID `15368`, with `strict` enabled; all review
rules, empty bypass allowances, and remaining flags must match. The GET response
wraps some booleans in `enabled` objects and includes server-generated fields.
It may also mirror check names in legacy `contexts`; the PUT uses `checks` to
retain App binding. Treat a mismatch as an incomplete application: investigate,
correct the payload or application, and read back again. Keep protection enabled
while resolving failures.

The two checks succeeded in real hosted runs and run on every pull request,
including stacked PRs, without branch/path filters or job conditions. Strict
status checks require an up-to-date base. Documentation `build`/`deploy` and
Pages publication run only on filtered main pushes or manual dispatch, so they
are excluded, as are Dependency Graph and manual-only checks. Requiring a
filtered or manual workflow can leave PR checks pending forever.

One eligible independent reviewer's approval is required for subsequent changes,
including policy changes. Stale approvals are dismissed, the latest reviewable
push needs approval from someone other than its pusher, and conversations must
be resolved. The PR author cannot supply the required approval. Local Codex
review under the author's CLI login does not satisfy GitHub's required approval.
Several other write-access maintainers exist; use an eligible independent
reviewer. There is no CODEOWNERS requirement or admin/actor/token bypass.
Normal eligible maintainers can merge through required PRs (`restrictions` is
null). Merge commits remain allowed; force pushes and deletion are prohibited,
and the branch remains unlocked for compliant updates.

Check-name or supplying-App changes require a coordinated policy update. Plan
the transition with an eligible maintainer so required checks remain available;
validate and merge the change, apply the updated payload, and verify read-back.
Do not disable protection to work around a missing check.

Run the portable regression tests with the existing test environment:

```sh
python -B -m pytest -q -p no:cacheprovider docker/tests/test_main_protection.py
```

These tests derive PR job names from the workflow YAML and verify the checked-in
payload. They do not query or change live GitHub settings.
