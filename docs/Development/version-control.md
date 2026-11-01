---
sidebar_position: 3
---

# Version Control & Pushing to the Stack

Now that you have made some changes here's how to push them to the stack:

**Never push directly to `main`.** `main` is what runs on the car, so everything goes in through a pull request.

## Make a branch

From the root of `wuair_v2`, start a branch for your change:

```bash
git checkout -b <your-branch-name>
```

Name it after what it does, with a prefix. We mostly use `feat/` and `fix/`:

```bash
git checkout -b feat/skidpad-planner
git checkout -b fix/state_est_docker_container
```

See the [Style Guide](/docs/Development/style-guide.md) for the full branching and commit message rules.

## Commit and push

Check what you changed, then commit it:

```bash
git status
git add <files you changed>
git commit -m "Add ROS parameter for max_speed"
```

Push to your branch, not `main`:

```bash
git push origin <your-branch-name>
```

## Open a pull request

Go to [WU-AI-Racing/wuair_v2](https://github.com/WU-AI-Racing/wuair_v2) on GitHub. It will usually offer a **Compare & pull request** button for the branch you just pushed. Open a pull request into `main`, say what you changed and how you tested it, and I'll review it :)

<!-- TODO: screenshot of the GitHub "Compare & pull request" banner / open pull request page
![Open a pull request on GitHub](/img/dev-guide/github-open-pr.png "Open a pull request")
-->

If you haven't opened one before, GitHub's guide walks through it: [Creating a pull request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request).
