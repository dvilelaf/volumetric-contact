# Publishing checklist

What the maintainer fills in before the repository goes public. Nothing here needs code changes.

1. **GitHub account.** The links to the repository and to the site carry the placeholder
   `OWNER` (`OWNER/volumetric-contact`, `OWNER.github.io`). Run once:

   ```
   scripts/set-owner.sh ACCOUNT      # --check lists what still has the placeholder
   ```

2. **Authors and identifiers.** In `CITATION.cff`: ORCID (commented line), the authors of the
   paper once agreed, and `preferred-citation` (title, DOI) when the paper exists. The paper's own
   TODO markers (`paper/sections/09-declarations.tex`) are separate.
3. **Pages.** Repository settings → Pages → Source: *GitHub Actions*. The `pages` workflow then
   publishes the paper PDF, the landing page and the viewer on every push to the default branch.
   Nothing is deployed from pull requests.
4. **Secrets scan.** `scripts/verify full` runs gitleaks on the whole history; CI runs it on every push.
5. **Citation metadata.** Optionally enable Zenodo for the repository to mint a DOI per release, and
   add it to `CITATION.cff` and the paper's data-availability statement.
6. **The container image** (`docker/Dockerfile`) is built by the manual `docker` workflow; it is
   too large for hosted runners' regular CI, which therefore covers the core library and the
   Python tools only. The `image` workflow (on a published release or by hand) builds the same
   Dockerfile and pushes it to `ghcr.io/dvilelaf/volumetric-contact-dev`, tagged with the toolchain
   (`drake-1.58.0-chrono-10.0.0-mujoco-4.0.0`) and the release tag. After the first push, set the
   package to public (Package settings → Change visibility) so that `scripts/dev.sh` can pull it
   anonymously when the image is missing locally.
