---
sidebar_position: 4
image: "/img/ros-logo.jpg"
---

# Windows: Clone and Build

:::tip[Reminder for help]

remember getting help is an email away. Don't spend 5 hours on a problem that might be a 5 minute fix. It is okay and expected for you to have problems. The link for getting in contact with us is [here](./help-me.md). You can always also contact your team lead.

:::

Prerequisites: [WSL](./installing-wsl.md), [Docker Desktop with WSL integration](./docker-download.md), and the [GitHub SSH key](../github-ssh-setup.md) set up inside WSL.

Run everything below **inside WSL** (Ubuntu).

```bash
mkdir -p ~/wuair && cd ~/wuair
git clone git@github.com:WU-AI-Racing/wuair_v2.git
cd wuair_v2
git submodule update --init --recursive
sudo usermod -aG docker $USER   # then close and reopen your WSL terminal
./scripts/build.sh              # takes a while the first time
```

Keep the repo in the WSL filesystem (`~/wuair`), not under `/mnt/c`; it is much faster.

:::danger[Important Warning]
If you get a permissions denied error, restart your terminal and/or Docker Desktop. If it persists, restart your computer. Docker Desktop can be finicky.
:::

:::important
Always build with `./scripts/build.sh`, not a bare `docker compose build`; the `controls` container needs the script to seed its `install/` folder.
:::

## Next: run it

Everything from here is identical on every OS: [Running the Stack](/docs/Development/running-the-stack.md).

import React from 'react';
import confetti from 'canvas-confetti';



export function ConfettiButton() {
  const shoot = () => {
    confetti({
      particleCount: 150,
      spread: 70,
      origin: { y: 0.6 }
    });
    const audio = new Audio('/wuair_docs/audio/confetti.mp3');
    audio.addEventListener('canplaythrough', () => audio.play());
  };
 const primaryBlue = "var(--ifm-color-primary)";
  return (
    <button
      onClick={shoot}
      style={{
        marginLeft: '8px',           
        padding: '4px 10px',         
        backgroundColor: primaryBlue,
        color: 'white',
        border: 'none',
        borderRadius: '4px',
        cursor: 'pointer',
        fontSize: '0.95rem',
        display: 'inline-block',
        verticalAlign: 'middle'
      }}
    >
      you deserve this!
    </button>
  );
}



<p>
  You did it! <ConfettiButton />
</p>
