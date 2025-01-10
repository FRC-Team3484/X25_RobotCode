# Organizing This Repository And Branches
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
Branches should be merged together using GitHub's Pull Requests feature

This essentially creates a space for discussion and code review before merging the code into a different branch

Any significant merge (feature into `development`, `development` into `testing`, `testing` into `main`, etc) should be done via Pull Requests if possible

Smaller merges (ones where code review is not required) can be completed using Pull Requests (which are then immediately merged) or by using `git merge <branch to merge into currently checked out branch>`

## File Organization
- Robot code related files should be placed inside the subfolder in this repository
- Any new documentation should be placed in the `docs` folder
- Any files needed for the `README` or this repository should be placed in the `repo` folder