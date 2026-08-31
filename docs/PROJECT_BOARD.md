# Road to Competition — Project Board

Board: **https://github.com/orgs/NASAKnights/projects/3**
Event: **Saturday–Sunday, October 24–25, 2026**

This doc explains how the board works and what our conventions are. If you're new to
GitHub Projects, read "The mental model" first — it's the part people get wrong.

---

## The mental model

**A GitHub Project is not a board. It's a database, and a board is one view of it.**

That single idea explains everything else. There is one set of rows, and many saved
lenses over them. The kanban board, the roadmap, and the "what's untested" table are all
reading the same data — changing an item in one changes it everywhere.

### Items — the rows

| Kind | Lives in | Use for |
|---|---|---|
| **Issue** | The repo | Real work. Has comments, labels, assignees, links to PRs. |
| **Pull request** | The repo | Auto-appears when a PR is opened. Links to the issue it closes. |
| **Draft issue** | The board only | Fast capture. No repo, no comments. Convert to a real issue later. |

Draft issues are the escape hatch for planning sessions: dump twenty ideas in two
minutes, sort them out afterward. Convert the ones that survive.

**Removing an item from the board does not close the issue.** The issue lives in the
repo; the board is a view. Closing the issue is what marks work finished.

### Fields — the columns

Every item carries values for the project's fields. Ours:

| Field | Type | Purpose |
|---|---|---|
| **Status** | Single select | Where it is in the pipeline. Drives the board columns. |
| **Subsystem** | Single select | Which class it belongs to. |
| **Work Type** | Single select | Model / Code / Tuning / Infra. |
| **Priority** | Single select | P0 Comp Blocker / P1 Should Have / P2 Nice to Have. |
| **Effort** | Number | Rough size: 1, 2, 3, or 5. Sums in Insights. |
| **Sprint** | Iteration | Which week it's planned for. |
| **Assignees**, **Labels**, **Milestone**, **Repository**, **Linked PRs** | Built-in | Come from the issue itself. |

`Status` isn't special magic — it's an ordinary single-select field that the Board layout
happens to group by. You can add or rename its options at any time, and every item using
an option updates automatically.

### Views — the lenses

A view saves a **layout** (Table, Board, or Roadmap) plus a filter, a grouping, a sort,
and which fields are visible. Different views, same rows.

### Workflows — the automation

Built-in rules, no YAML required, under **⋯ → Settings → Workflows**. They can set
`Status` and add items. They **cannot** set custom fields like Subsystem or Priority — a
human does that at triage.

---

## Architecture reference

The Subsystem field mirrors the Executable UML model:

```
FieldData   ──field layout / alliance / FMS──> Input
Vision/Pose ──pose─────────────────────────────> Input
Autonomous  ──direction────────────────────────> Input
Driver                                         > Input
                                                  │
                                                  ▼
                                              Robot State ──> Swerve Drive
                                                             ├> Intake
                                                             ├> Extender
                                                             ├> Indexer
                                                             └> Dumper
```

- **Robot State** is the coordinator. It consumes Input and drives the mechanism classes.
- **Input** is the single funnel. Teleop and autonomous take the same path in, so a fix in
  Robot State works in both.
- **Intake** has two motors: one extends/retracts the arm, one drives balls into the hopper.
- **Extender** raises and lowers to increase hopper volume.
- **Indexer** moves balls from the hopper into the Dumper.
- **Dumper** fires with a flywheel and hood.
- **Build/Tooling** is *not* a model class. It's a board bucket for gradle, vendordeps, CI,
  deploy scripts, xUML tooling, and docs — work that would otherwise get misfiled under
  whatever subsystem it happened to touch.

---

## Status columns

```
Triage → Todo → In Progress → In Review → Needs Robot Test → Comp Ready
```

| Column | Means | Who moves it |
|---|---|---|
| **Triage** | Filed, not yet scoped. No Subsystem/Priority/Effort set. | Automation, on file |
| **Todo** | Scoped, prioritized, assigned. Ready to pick up. | Mentor/lead at triage |
| **In Progress** | Someone is actively working it. | Whoever picks it up |
| **In Review** | PR is open. | Automation, on PR open |
| **Needs Robot Test** | Merged, but **nobody has run it on the robot**. | Automation, on PR merge |
| **Comp Ready** | Verified running on the actual robot. | A human who watched it work |

### Why "Needs Robot Test" exists

Because merged code that has never run on hardware looks exactly like working code, and
you find out which is which at the worst possible moment. Two of the commits on this repo
right now are literally titled *"need to test."*

**The rule: only a person who watched it run on the real robot moves a card to Comp Ready.**
Not the author, not because CI passed, not because it worked in sim. Every issue template
asks "how will we verify this on the robot?" — that's the check you run.

The night before load-in, open the **⚠️ Untested on Robot** view. Whatever's in it is the
risk you're bringing to the event.

### Blocked work

There's no Blocked column — blocked work is still someone's In Progress work, and hiding
it in a parking lot means nobody chases it. Add the **`blocked`** label instead and say in
a comment what you're waiting on. It shows up as a red label in every view.

---

## Views

| View | Layout | Shows |
|---|---|---|
| **Triage** | Table | `Status = Triage`. The mentor queue. Set Subsystem, Priority, Effort, Sprint, assignee, then move to Todo. |
| **Sprint Board** | Board by Status | Current sprint only. The daily driver. |
| **⚠️ Untested on Robot** | Table | `Status = Needs Robot Test`. The pre-event checklist. |
| **Comp Blockers** | Table | P0 items not yet Comp Ready. |
| **By Subsystem** | Board by Subsystem | Where work is piling up, and which classes are modeled but not built. |
| **Roadmap** | Roadmap by Sprint | Timeline, grouped by Milestone. |

---

## Sprints

Eight one-week sprints, Monday to Sunday.

| Sprint | Dates | |
|---|---|---|
| 1 | Aug 31 – Sep 6 | Modeling |
| 2 | Sep 7 – Sep 13 | Modeling → **M0 due Sep 13** |
| 3 | Sep 14 – Sep 20 | Trim |
| 4 | Sep 21 – Sep 27 | Rebuild |
| 5 | Sep 28 – Oct 4 | Rebuild |
| 6 | Oct 5 – Oct 11 | Bring-up |
| 7 | Oct 12 – Oct 18 | Driver practice, tuning |
| 8 | Oct 19 – Oct 25 | **Freeze Oct 22**, event Oct 24–25 |

The phase labels are intent, not commitments. Re-cut them once the model tells you how
much rebuild there actually is.

## Milestones

Milestones are a **repo** feature, separate from the project — they give a completion bar
and a hard date. Set one on every issue.

| Milestone | Due |
|---|---|
| M0 – Executable UML Model Complete | Sep 13 |
| M1 – Code Trimmed to Baseline | *undated* |
| M2 – Subsystems Rebuilt & Moving | *undated* |
| M3 – Driver Ready | *undated* |
| M4 – Auto & Tuning Complete | *undated* |
| M5 – Code Freeze | Oct 22 |

M1–M4 are deliberately undated. A date you invented before knowing the rebuild size is a
guess wearing a deadline's clothes. Add real dates after M0.

---

## Conventions

1. **File through a template.** Feature/Task, Bug, or Tuning. Blank issues are off.
2. **Everything lands in Triage.** Automation adds it; a mentor scopes it.
3. **Triage sets four fields:** Subsystem, Work Type, Priority, Effort. Then Todo.
4. **Assign yourself before you start** and move it to In Progress. One In Progress item
   per person — if you're blocked, label it `blocked` and pick up something else.
5. **Reference the issue in your PR** (`Closes #42`) so it links automatically.
6. **Don't close an issue until it's robot-tested.** Closing sets Comp Ready.
7. **Say what you did in the issue** when you verify — "ran it on the robot 10/12, arm
   extends and retracts clean." Future you will want to know.

---

## Setting up the board (one-time, web UI)

Fields, milestones, labels, and templates are already done. These four things have no API
and must be clicked.

### 1. Configure the Sprint field

Project → **⋯ → Settings** → **Sprint** (under Fields).

Set **Starts on** Monday, **Duration** 1 week, then **Add iteration** until you have 8,
running Aug 31 through Oct 25. Rename them Sprint 1–8 if you like.

### 2. Turn on workflows

Direct link: **https://github.com/orgs/NASAKnights/projects/3/workflows**

Or: project → **⋯** (top right, next to the search box) → **Settings** → **Workflows** in
the left sidebar. It's below Details, Fields, Views, and Manage access — easy to scroll
past.

The workflows already exist and are toggled **off**. You don't create them; you click one,
set its action in the right-hand panel, and flip the **Enable** toggle at the top. Nothing
happens until you flip it.

| Workflow | Setting |
|---|---|
| **Auto-add to project** | Repo `2026-RumbleBot`, filter `is:issue is:open` |
| **Item added to project** | Set Status → **Triage** |
| **Pull request merged** | Set Status → **Needs Robot Test** |
| **Item closed** | Set Status → **Comp Ready** |
| **Auto-archive items** | `is:closed updated:<@today-14d` |

Two gotchas:

- If the repo picker under Auto-add is empty, the project doesn't have access to the repo
  yet — set that under **Settings → Manage access** first.
- Auto-add only catches issues filed *after* you turn it on. Add existing ones by hand once.

### 3. Build the views

Each view is a tab at the top. Click **+ New view**, then set layout, filter, grouping,
and sort in the view's **⌄** menu. Rename by double-clicking the tab. **Save** when the tab
shows unsaved changes.

| # | Name | Layout | Filter | Group | Sort |
|---|---|---|---|---|---|
| 1 | Triage | Table | `status:Triage` | — | Created ↑ |
| 2 | Sprint Board | Board | `sprint:@current` | Status | Priority ↑ |
| 3 | ⚠️ Untested on Robot | Table | `status:"Needs Robot Test"` | Subsystem | Priority ↑ |
| 4 | Comp Blockers | Table | `priority:"P0 Comp Blocker" -status:"Comp Ready"` | Status | — |
| 5 | By Subsystem | Board | `-status:"Comp Ready"` | Subsystem | Priority ↑ |
| 6 | Roadmap | Roadmap | — | Milestone | — |

On the Roadmap view, open **⌄ → Date fields** and set the marker to **Sprint** so items
place on the timeline.

Filter syntax notes: quote values containing spaces, prefix with `-` to exclude,
`@current` means the active iteration, `@me` means you. `no:assignee` and `no:status` are
useful for catching items that fell through triage.

### 4. Set the board's README

Project → **⋯ → Settings → README**. Paste a link back to this file so people arriving at
the board find the conventions.
