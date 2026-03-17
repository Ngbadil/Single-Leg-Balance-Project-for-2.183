# Single-Leg-Balance-Project-for-2.183

For the time being, everyone will have their own branch they can commit changes to. Once changes are reviewed and tested we will merge branches to main.


1. Clone the repository
Use this the first time you download the project to your computer.

git clone https://github.com/Ngbadil/Single-Leg-Balance-Project-for-2.183.git
cd Single-Leg-Balance-Project-for-2.183

2. Check the current branch and status
This shows which branch you are on and whether you have any uncommitted changes.

git status
git branch


3. Create and switch to your own branch
Before making changes, create a new branch so your work stays separate from the main branch.
git switch -c your-branch-name

4. Make your changes
Edit the files you need in the repository.To see what changed:

git status
git diff
5. Stage your changes

Stage all changed files:
git add .

Or stage a specific file:
git add path/to/file

6. Commit your changes
Write a clear commit message describing what you changed.

git commit -m "Describe your changes here"

7. Pull the latest changes from the remote repository
Before merging or pushing, make sure your local copy is up to date.
If you are updating the main branch:

git switch main
git pull origin main



8. Merge the latest main branch into your branch
After updating main, switch back to your branch and merge the latest changes into it.

git switch your-branch-name
git merge main

10. Merge your branch back into main
Once your work is finished and tested, merge it into main.

git switch main
git pull origin main
git merge your-branch-name
git push origin main


This project builds on an existing experimental dataset collected from younger and older participants performing the One-Legged Stand Test (OLST), a widely used clinical assessment of balance, and extends prior work that conducted a frequency-dependent intersection point height (zIP) analysis. The goal of this project is to develop a double-inverted pendulum (DIP) model of human balance during OLST and integrate a Linear Quadratic Regulator (LQR) optimal control framework to better interpret observed zIP control patterns and their relationship to fall risk.

We will implement a sagittal-plane DIP model in MATLAB and apply an LQR optimal control framework to identify best-fit biomechanical and control parameters at both the population and individual participant levels. In particular, we aim to recover interpretable parameters such as joint torque weighting (β) and joint noise weighting (σ), which may help link zIP curves to underlying central nervous system control strategies and provide a more mechanistic interpretation of balance control differences.

We will also examine how variations in model parameters, such as center of mass (CoM) height and moment of inertia distribution, affect balance control, allowing us to simulate demographic differences (e.g., age-related changes) and task-specific characteristics. The framework will ultimately be extended to the frontal plane to capture mediolateral balance dynamics during OLST.

By combining experimental data, zIP analysis, and optimal control modeling, this project aims to establish a more neuro-biomechanically grounded and interpretable framework for understanding balance control and its relationship to fall risk.
