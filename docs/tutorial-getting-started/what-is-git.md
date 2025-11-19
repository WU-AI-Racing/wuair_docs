---

sidebar_position: 1

---

# What is Git?



GitHub is a **Versioning Control Software** which allows people like us to collaborate on code without causing more problems than we solve. With this system (and by adhering to good practices using it) we can create, share and collaborate on projects much larger than what any one of us could create individually and still be effective. 



:::tip[Optional Content Ahead]



We won't be assuming a technical background when writing these first few pages of documentation, so anything you feel comfortable with skipping here you can. We will go over simple setup here, but the most accurate instructions will be related to your OS (Operating System, e.g. Windows, Mac, Linux). Take care to support these instructions with the relevant ones for your OS, also detailed in the documentation. 



No matter who you are, **take care to brush up on good coding practices.** These are detailed in the last section.



:::



## So how do I get started?



First, make a GitHub account [here](https://github.com/). You should affiliate the account with the email you received the slack invitation for WUAIR. 



Then you'll need to install the git CLI. Many computer come with git preinstalled, but be wary that may not be the case. [This](https://cli.github.com/) website should be able to help.



Then, you'll need to create an SSH key.



## Your first SSH key:



You can skip this if you just want to learn about GitHub, and maybe don't feel ready yet to tackle what's below. 



:::danger[Warning]



This advice is agnostic to OS and other requirements. Make sure you are following the instructions your OS-specific documentation asks you to. For example, Windows instructions will have you use WSL and the Linux instructions. 



:::



1. For maximal convenience, name your key "git_key" (to match subsequent commands) and do not add a passphrase. **This is not security advice, do not do this in real life**, we should **never use this key in any public facing automation**. You have been warned. Follow [this tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent?platform=linux) to generate the key. 

2. To add the SSH key to your Github Account, use [this tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) (remember our key is named git_key, so replace any "id_somenumber" with "git_key").



## Creating the SSH Agent

3. If you did every step right, you might also have to instantiate the SSH-Agent (the process that facilitates ssh connections with GitHub using your generated keys). 



4. to check if the commands succeeded, check for an active agent.



:::tip[What commands do I run?]



These instructions are purposefully vague. Use your **OS-specific** documentation for exact commands.



:::





## Cloning the repository

We will now be cloning the repository (repo) using an SSH connection. Repositories are essentially the folders that hold all our code, and are maintained online on github.com and by you on your local machine. The first time you do this, ssh will ask you to trust github, and github will give you some information as proof of their authenticity. You can safely write "yes" and move on.

```bash

git clone git@github.com:WU-AI-Racing/wuair_system.git

cd wuair_system && git checkout your_branch_here # move to desired branch

```



You just set up an environment using github! Now lets look at how we can use it effectively.



## Using GitHub



There are many significant commands in git, so let's start slow. We will be using Github's CLI (command line interface).



The first three to learn are how we get code from our computers onto the website:

```bash

git add .

git commit -m "message here"

git push

```



1. `git add` will tell git which changes to track. The dot will effectively tell git to track all of our changes. This is the most common use case. 



2. `git commit` will tell git to take all of our tracked changes, add a title (determined by our message, it is important to be succinct and direct in this message) and stage the commit to be 'pushed', which gets it onto Github.com for everyone to see.



3. `git push` pushes the commit to the 'branch' you are working in.



Finally, `git pull` will take the most recent pushes to our branch and pull them into our local workspace. Think of it like updating our computer with everyone on the branches' most recent changes.



## Branches, pull requests, merges???





Okay, now we're getting to the good part. What's a branch?



### Branches

Github provides branches, in other words methods for us to be working on separate features in code without stepping on each other's feet. As much as I would love to link a conveniently perfect explanation, most of them suck / suffer from being overly technical



In practice, each team will have one main branch. We will never push code we know may not work yet there. Only your best finished code goes to main. Your team will then have at least one separate 'branch' for new changes. This way, you can still share code with people on your team without potentially breaking something important for every other team. 



The command to visit other branches is 



```bash

git checkout "branch_name"

```



The command to make a new branch is



```bash
git branch -b "new_branch_name"
```



### Pull Requests and Merges

Okay, lets say we go to a new branch, work on our new feature, add, commit and push our code, and its tested and works great. How do we move this code to our main branch? This is where pull requests and merges come into place. 



After we push to a branch and are ready to merge our changes into the main branch, we have to make a **Pull Request**. Pull requests are made on the github website, and you'll get a popup on the website when new PR (pull request) happens or a new push was just made, and you can make a PR. 



You'll be asked to add a description of the PR when you go to make one (after you push and are ready to merge, on github.com). This should be a lengthy, descriptive documentation of what feature(s) were implemented, how and why. You should add anything additional you downloaded (dependencies) and describe why they are necessary. After your PR is reviewed, it will be merged into the main branch (your edits will be transfered over). 



### The dreaded Merge Conflict



Merge conflicts happen when github does not know how to resolve the differences between your version of the repository and the version online. This could happen because you edited a file and someone else did too, or because changes were made and you did not pull before you staged your commit and tried to push (always `git pull` before any `git add, commit or push`), or a bunch of other reasons. 











