# Team Git Workflow

This section explains how to clone the **visionnav-ugv** repository, create your own branch using your name, make changes, and push your work without directly modifying `main`.

## 1. Clone the Repository

```bash
git clone https://github.com/haricharan39/visionnav-ugv.git
cd pathrage
```

Check the repository:

```bash
git status
```

---

## 2. Create Your Own Branch

**Do not work directly on `main`.**

Create a branch using **your name**:

```bash
git checkout -b <your-name>
```

Example:

```bash
git checkout -b hari
```

Check your branch:

```bash
git branch
```

You should see:

```text
* hari
  main
```

The `*` shows your current branch.

### Branch Naming Rule

**Branch names must be the member's name.**

Examples:

```text
hari
rahul
arjun
sai
```

---

## 3. Work on Your Branch

Make your changes normally.

Check your changes:

```bash
git status
```

Review them:

```bash
git diff
```

---

## 4. Commit Your Changes

Add your changes:

```bash
git add .
```

Commit them:

```bash
git commit -m "Describe your changes"
```


## 5. Push Your Branch

Push your name-based branch to GitHub:

```bash
git push -u origin <your-name>
```

Example:

```bash
git push -u origin hari
```

Your branch will now appear on GitHub.


# Updating Your Branch

Before starting work, get the latest changes from `main`.

Make sure you are on your own branch:

```bash
git checkout <your-name>
```

Get the latest changes:

```bash
git fetch origin
```

Update your branch:

```bash
git merge origin/main
```

If there are merge conflicts, ask the team before resolving complicated conflicts.

---

# Recommended Daily Workflow

### Start working

```bash
git checkout <your-name>
git fetch origin
git merge origin/main
```

### After making changes

```bash
git status
git add .
git commit -m "Describe your changes"
git push
```

---

# Branch Structure

```text
main
 │
 ├── hari
 │    └── my file changes
 │
 ├── member2
 │    └── Member 2's changes for main
 │
 └── member3
      └── Member 3's changes from main
```

Each member works .

Changes are merged into `main` through Pull Requests.

---

# Important Rules

### 1. Never push directly to `main`

Always work on your own name-based branch.

### 2. Branch name = your name

### 3. Keep your branch updated

Before starting work:

```bash
git fetch origin
git merge origin/main
```

### 4. Do not commit generated files

Normally, these should not be committed:

```text
build/
install/
log/
__pycache__/
```

---

# GitHub Authentication

Each team member should use **their own GitHub account**.

**Never share your GitHub password.**

If the repository is private, the repository owner must give each team member access to the repository.

---

# Quick Reference

```bash
# Clone
git clone https://github.com/haricharan39/visionnav-ugv.git
cd visionnav-ugv

# Create your branch
git checkout -b <your-name>

# Work...

# Check changes
git status
git diff

# Stage
git add .

# Commit
git commit -m "Describe your changes"

# Push
git push -u origin <your-name>

# Update from main
git fetch origin
git merge origin/main
```

## Simple Workflow

```text
       main
        │
   ┌────┼────┐
   ↓    ↓    ↓
 hari  rahul  arjun
   │    │    │
   │ changes │
   │    │    │
   └────┼────┘
        ↓
   Pull Request
        ↓
       main
```

**`main` = shared stable branch**

**`your-name` = your personal branch**

**Pull Request = merge your work into `main`**
# GitHub SSH Setup

Each team member must connect **their own GitHub account** using an SSH key.

> **Never share your SSH private key or GitHub password.**

## 1. Check for an Existing SSH Key

```bash
ls -al ~/.ssh
```

If you already have a key such as `id_ed25519`, you can use it. Otherwise, generate a new one.

## 2. Generate an SSH Key

```bash
ssh-keygen -t ed25519 -C "your-github-email@example.com"
```

When prompted:

```text
Enter file in which to save the key:
```

Press **Enter** to use the default:

```text
~/.ssh/id_ed25519
```

Enter a passphrase when asked. It is recommended for security.

## 3. Start the SSH Agent

```bash
eval "$(ssh-agent -s)"
```

Add your private key:

```bash
ssh-add ~/.ssh/id_ed25519
```

Verify:

```bash
ssh-add -l
```

## 4. Copy Your Public Key

**Only copy the `.pub` file. Never share `id_ed25519`.**

```bash
cat ~/.ssh/id_ed25519.pub
```

Copy the entire output. It should look like:

```text
ssh-ed25519 AAAA... your-github-email@example.com
```

## 5. Add the Key to GitHub

Open GitHub and go to :

**Settings → SSH and GPG keys → New SSH key**

Enter:

```text
Title: Ubuntu 22.04 - <your-name>
Key type: Authentication Key
Key: <paste your public key>
```

Click **Add SSH key**.

## 6. Test the Connection

Run:

```bash
ssh -T git@github.com
```

The first time, GitHub may ask:

```text
Are you sure you want to continue connecting (yes/no/[fingerprint])?
```

Type:

```bash
yes
```

A successful connection will show a message similar to:

```text
Hi <haricharan39>! You've successfully authenticated,
but GitHub does not provide shell access.
```

Your SSH connection is now ready.

---

# 7. Clone the Repository Using SSH

Use the SSH URL instead of HTTPS:

```bash
git clone git@github.com:haricharan39/visionnav-ugv.git
cd visionnav-ugv
```

## 8. Create Your Personal Branch

Your branch name must be **your name**:

```bash
git checkout -b <your-name>
```

Example:

```bash
git checkout -b hari
```

## 9. Push Your Work

After making changes:

```bash
git add .
git commit -m "Describe your changes"
git push -u origin <your-name>
```

Example:

```bash
git push -u origin hari
```

After the first push, future pushes only require:

```bash
git push
```

---

### Security

```text
id_ed25519       → PRIVATE — NEVER SHARE
id_ed25519.pub   → PUBLIC  — ADD TO GITHUB
```

Every one should generate **their own SSH key** and add it to **their own GitHub account**.
