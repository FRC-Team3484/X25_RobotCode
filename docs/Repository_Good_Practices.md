# Repository Good Practices
This page documents good practices for how to organize files in this repository, how to merge branches together, and when to create new branches

## Branches
This repository contains several branches that are used in different stages throughout the season

- First, branches should be created for new larger features that are in progress (eg `launcher`, `auton`, `vision`, `commands`). Once these features have been implemented, they should be merged into `development` and then tested
- `development` - This branch is used to merge other individual branches together and for adding smaller features
- `testing` - Once all the features are merged into the `development` branch, this branch is used to test the code. Any necessary changes and fixes should be committed here
- `main` - Once the code in `testing` has been fully tested and is ready for competition, it should be merged into `main`. This code is ready for competition and shouldn't contain any experimental features or untested changes


### Branch Naming Conventions
Branches should be all lowercase and any spaces should be replaced with `-`

For example, `development`, `vision-experiment`, `launcher`, `auton`, `commands`

## Merging Branches
- Branches should be merged together using GitHub's Pull Requests feature
- This essentially creates a space for discussion and code review before merging the code into a different branch
- Any significant merge (feature into `development`, `development` into `testing`, `testing` into `main`, etc) should be done via Pull Requests if possible
- Smaller merges (ones where code review is not required) can be completed using Pull Requests (which are then immediately merged) or by using `git merge <branch to merge into currently checked out branch>`
- Any merges into `development`, `testing`, or `main` should only be done if the code is building with no errors

## File Organization
- Robot code related files should be placed inside the subfolder in this repository
- Any new documentation should be placed in the `docs` folder
- Any files needed for the `README` or this repository should be placed in the `repo` folder

## Good Commit Practices

Follow [How to use Git](docs/How_to_Use_Git.md) for general Git usage instructions. This section demonstrates good commit and commit message practices
- **Commit often** - Make a new commit whenever you complete a feature or working part of a feature (creating a new command, updating auton paths, modifying a subsystem, etc)
  - Don't commit to `development`, `testing`, or `main` unless ythe code builds. Commits to feature branches can be pushed if the code doesn't build
  - Don't commit multiple large changes or features in one commit. If we need to rollback or see when something changed, it's easier if each commit only changes one major feature
- **Prefix your commit message** - Prefixing your commit message allows for quick filtering and scanning to find your change
  - Common prefixes include `feat:` (feature), `fix:` (bugfix), `docs:` (documentation), `chore:` (other changes), `refactor:` (refactoring), `test:` (tests), `style:` (style changes)
  - For example, "feat: Add launcher command", "fix: Fix unit conversion bug in vision", "docs: Update README", etc.
- **Use descriptive and simple commit messages** - Commit messages should be simple and descriptive
  - For example, "feat: Update auton paths", "docs: Update Repository Good Practices guide", "fix: Fix logic bug in launcher command", etc.
  - Commit message length should be under 72 characters to make them quick to read
  - If more characters are needed, break the commit message into multiple lines or adding to the commit message description