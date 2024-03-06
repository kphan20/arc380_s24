This is a basic workflow for git:

If you're using VS Code, a lot of these commands can be done with the GUI (there should be a sidebar option called version control on the left). Even with that, having this reference can be useful if you want to get a general understanding of git and/or to use a terminal instead.

1. Create your own branch with `git checkout -b [BRANCH NAME]`. This creates a new branch based off of the branch you are currently on. You can check which branch you are on with `git status` and change to a branch using `git checkout [BRANCH NAME]` (no -b option).

2. After you've made some changes, you need to let git know which changes you want to commit since you sometimes don't want to include everything in one commit.

`git add [FILE NAME]` adds all changes in one file.
`git add .` adds everything in the current directory you are in.
`git add -p [FILE NAME]` allows you to open an interactive window and choose which changes within a file you want to stage.

3. You can commit using `git commit -m [MESSAGE]`. It's usually useful to add a descriptive message as the number of commits grows.

Before pushing your code, it's usually good practice to pull changes that other people might have made to make sure your code is compatible. To pull changes from a remote branch to your local one, run `git pull origin [REMOTE BRANCH]`. This may cause merge conflicts which you'll need to resolve. We also should work out more specifics about this.

4. You can push your commits to Github. If it's the first time pushing on the current branch you're on, then use `git push -u origin [BRANCH NAME]`. Otherwise, you can just use `git push`.
