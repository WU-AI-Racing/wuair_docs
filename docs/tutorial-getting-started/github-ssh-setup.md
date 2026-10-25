---
sidebar_position: 0
---

# GitHub SSH Setup (everyone)

Do this once, on every OS, before the OS-specific Docker and clone steps. We clone `wuair_v2` over SSH.

:::important Windows: do this inside WSL
Open a WSL terminal (run `wsl -d Ubuntu-22.04` in PowerShell) and run every command below there, not in PowerShell. The key must live in the WSL home folder (`~/.ssh`), because that is where you will clone and build. Install WSL first: [Installing WSL](./windows-startup/installing-wsl.md).
:::

## 1. Install git

- **Linux / WSL (Ubuntu):** `sudo apt update && sudo apt install -y git`
- **Mac:** `xcode-select --install` (or `brew install git`)

Set your identity:

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"
```

## 2. Generate a key

```bash
ssh-keygen -t ed25519 -C "you@example.com"
```

Press Enter to accept `~/.ssh/id_ed25519`. A passphrase is optional. If you already have a key you use with GitHub skip this and use it.

## 3. Add it to GitHub

```bash
cat ~/.ssh/id_ed25519.pub
```

Copy the output, then on GitHub go to **Settings → SSH and GPG keys → New SSH key**, paste it and save. (Official guide: [adding a new SSH key](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account).) You also need to be in the **WU-AI-Racing** GitHub org; ask your team lead for an invite.

## 4. Test

```bash
ssh -T git@github.com
```

The first time, type `yes` to trust GitHub. You should see `Hi <username>! You've successfully authenticated`. If you get `Permission denied (publickey)`, the key was not added correctly; redo step 3.

## 5. Continue with your OS

| OS | Next |
|----|------|
| Windows | [Installing WSL](./windows-startup/installing-wsl.md), then [Docker](./windows-startup/docker-download.md), then [Clone and build](./windows-startup/using-docker-plus-ros.md) |
| Mac | [Mac Setup](./mac-stetup.md) |
| Linux | [Linux Setup](./software-installation.md) |

