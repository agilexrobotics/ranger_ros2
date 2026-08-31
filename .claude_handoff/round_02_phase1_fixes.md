# Round 02 — Phase 1 follow-up: ground-plane fix + open-question resolutions

## Summary

Rebased `phase-1-description` from `humble` onto `jazzy` (clean — no
conflicts). Applied the architect's three resolutions: changed RViz Fixed
Frame from `base_link` to `base_footprint` so the robot sits on the grid
plane; dropped `config` from the package's `install(DIRECTORY ...)` list
and removed the now-empty `config/` directory; updated `package.xml`
maintainer to the host git identity. The xacro is unchanged (URDF math
was already correct), the package rebuilds cleanly, and the installed
RViz config carries the new fixed frame. Two new commits on
`phase-1-description`. **Important:** the rebase replayed 9 humble-only
upstream commits (touching `ranger_base/`, `ranger_bringup/`, root
`README.md`) that do not exist on `jazzy`; they are now part of this
branch — see Open Questions for whether they should stay.

## Diff

Rebase output (clean, no conflicts):

```
branch 'jazzy' set up to track 'origin/jazzy'.
Rebasing (1/11)…(11/11)
Successfully rebased and updated refs/heads/phase-1-description.
```

The 11 replayed commits are:

```
9d081fe docs(handoff): round 01 description package import          (mine, R01)
05e6f95 feat(sim): add ranger_mini_v3_description package (phase 1) (mine, R01)
91516c0 update the CalculateSteeringAngle calculation method        (smalleha)
7e17974 Fix bugs related to movement                                (smalleha)
8c697b8 fixd launch file bug                                        (agilexrobotics)
9de3b92 add rm3 params                                              (agilexrobotics)
a4949b9 Update README.md                                            (agilexrobotics)
da48f2b change publish_odom_tf parameter                            (agilexrobotics)
f2b1574 fixed bug and add launch.py                                 (agilexrobotics)
1b90770 add motor_angles and motor_angles                           (agilexrobotics)
cd48b3e update driver                                               (agilexrobotics)
```

The 9 non-mine commits touch only `ranger_base/`, `ranger_bringup/`, and
`README.md` — files that the bootstrap policy says I must not modify.
They came in via the literal `git rebase jazzy` invocation; I did not
edit them. Flagged in Open questions (1).

This round's source commit (`dd72af7`):

```diff
commit dd72af7b44fb4978c5b116acce6470bd431a3a23
Author: Shuaib Olanrewaju <solanrewaju2020@fau.edu>
Date:   Tue May 12 00:12:11 2026 -0400

    fix(sim): ground rviz on base_footprint; clean up phase 1

diff --git a/ranger_mini_v3_description/CMakeLists.txt b/ranger_mini_v3_description/CMakeLists.txt
index 3b46d34..e2d58ce 100644
--- a/ranger_mini_v3_description/CMakeLists.txt
+++ b/ranger_mini_v3_description/CMakeLists.txt
@@ -5,7 +5,7 @@ find_package(ament_cmake REQUIRED)
 
 # Ship every asset the robot description, RViz, and launch system need.
 install(
-  DIRECTORY urdf meshes rviz launch config
+  DIRECTORY urdf meshes rviz launch
   DESTINATION share/${PROJECT_NAME}
 )

diff --git a/ranger_mini_v3_description/package.xml b/ranger_mini_v3_description/package.xml
index edc6ca0..f1f729c 100644
--- a/ranger_mini_v3_description/package.xml
+++ b/ranger_mini_v3_description/package.xml
@@ -7,7 +7,7 @@
     URDF / xacro description and meshes for the AgileX Ranger Mini v3.
     Ported from agilexrobotics/ugv_gazebo_sim (ROS 1) to ROS 2 Jazzy + Gazebo Harmonic.
   </description>
-  <maintainer email="you@example.com">you</maintainer>
+  <maintainer email="solanrewaju2020@fau.edu">Shuaib Olanrewaju</maintainer>
   <license>BSD-3-Clause</license>

diff --git a/ranger_mini_v3_description/rviz/display.rviz b/ranger_mini_v3_description/rviz/display.rviz
index 85c7179..0fc8f33 100644
--- a/ranger_mini_v3_description/rviz/display.rviz
+++ b/ranger_mini_v3_description/rviz/display.rviz
@@ -31,7 +31,7 @@ Visualization Manager:
       Show Names: true
       Marker Scale: 0.3
   Global Options:
-    Fixed Frame: base_link
+    Fixed Frame: base_footprint
     Background Color: 48; 48; 48
```

The empty `config/` directory was also `rmdir`-ed off disk; git did not
track it (empty dirs aren't versioned), so this does not appear in the
diff.

## New files (full content)

### .claude_handoff/.gitignore

```gitignore
# Editor / viewer metadata that shouldn't pollute the handoff trail.
.obsidian/
.vscode/
.idea/
```

(Created to keep `git add -A` from staging an Obsidian sidecar dir
that had appeared in `.claude_handoff/.obsidian/` — see Deviations.)

No other new files this round; all other changes are edits to files that
already exist (and whose full content was captured in
`round_01_description.md`). The current state of the three edited files,
post-fix, is:

### ranger_mini_v3_description/CMakeLists.txt (post-fix)

```cmake
cmake_minimum_required(VERSION 3.8)
project(ranger_mini_v3_description)

find_package(ament_cmake REQUIRED)

# Ship every asset the robot description, RViz, and launch system need.
install(
  DIRECTORY urdf meshes rviz launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### ranger_mini_v3_description/package.xml (post-fix maintainer line only)

```xml
<maintainer email="solanrewaju2020@fau.edu">Shuaib Olanrewaju</maintainer>
```

### ranger_mini_v3_description/rviz/display.rviz (post-fix Global Options block only)

```yaml
  Global Options:
    Fixed Frame: base_footprint
    Background Color: 48; 48; 48
```

## Verification output

### git rebase jazzy

```
branch 'jazzy' set up to track 'origin/jazzy'.
Rebasing (1/11)…(11/11)
Successfully rebased and updated refs/heads/phase-1-description.
```

(Note: a local `jazzy` branch did not exist before this round — only the
remote `origin/jazzy`. I created it via `git branch jazzy origin/jazzy`
so the literal `git rebase jazzy` command from the prompt would resolve.
Logged in Deviations.)

### xacro ranger_mini_v3_description/urdf/ranger_mini_v3.xacro > /tmp/ranger_check2.urdf

```
exit=0
275 /tmp/ranger_check2.urdf
```

(Identical line count to round 01 — confirms the xacro is untouched.)

### colcon build --packages-select ranger_mini_v3_description --symlink-install

```
Starting >>> ranger_mini_v3_description
Finished <<< ranger_mini_v3_description [0.32s]

Summary: 1 package finished [0.40s]
```

### grep "Fixed Frame" install/.../rviz/display.rviz

```
    Fixed Frame: base_footprint
```

### Manual re-verification steps for the operator

The agent does not launch GUI applications. To re-verify visually after
the ground-plane fix:

```bash
# In a new terminal:
cd ~/agilex_ws
source install/setup.bash
ros2 launch ranger_mini_v3_description display.launch.py
```

Expected outcome (changed bits in **bold**):

- The robot now sits **on top of** the grid plane, not partially below
  it. Tires touch the grid; chassis floats ~0.31 m above it.
- The TF display shows a **`base_footprint`** label at ground level,
  directly beneath `base_link`. The fixed-frame readout in the RViz
  bottom-left says `base_footprint`.
- The eight slider joints in the `joint_state_publisher_gui` window
  (`fl/fr/rl/rr_steering_joint`, `fl/fr/rl/rr_wheel`) still rotate the
  corresponding links exactly as they did in Round 01.

If anything looks off, capture an RViz screenshot for the architect.

## Deviations

- **`git rebase jazzy` carried 9 humble-only upstream commits onto
  `phase-1-description`.** These touch `ranger_base/`,
  `ranger_bringup/`, and the root `README.md` — packages the bootstrap
  forbids me from modifying. I did not author or edit them; they were
  replayed by git as part of executing the literal command. The rebase
  was clean (no conflicts) so no manual decision was needed at the time.
  See Open question (1).

- **Created a local `jazzy` branch** (`git branch jazzy origin/jazzy`)
  before the rebase. The literal command `git rebase jazzy` from the
  prompt fails with `fatal: invalid upstream 'jazzy'` if no local
  branch by that name exists. The minimum-change interpretation was to
  create the local tracking branch so the prompt's command would run
  unchanged. The alternative (`git rebase origin/jazzy`) would have
  worked too but deviated from the literal command.

- **Created `.claude_handoff/.gitignore`** that excludes `.obsidian/`,
  `.vscode/`, `.idea/`. An Obsidian sidecar directory had appeared
  inside `.claude_handoff/.obsidian/` between rounds (presumably the
  operator viewing the markdown), and a literal `git add -A` would
  have committed it. The gitignore keeps the handoff trail clean
  without affecting non-Obsidian users. The `.gitignore` is staged in
  the round-02 handoff commit (it's a handoff-trail artifact, not a
  package-fix artifact).

- **Source-fix commit was made via `git add <three files>`** instead of
  the literal `git add -A` from the prompt. Reason: `-A` would have
  staged `.claude_handoff/.gitignore` into the source-fix commit, which
  doesn't fit the commit message `fix(sim): ground rviz on base_footprint;
  clean up phase 1`. The .gitignore goes into the handoff commit instead.

## Open questions

1. **Should the 9 humble-only upstream commits be dropped from
   `phase-1-description`?** They modify packages outside this port's
   scope (`ranger_base/`, `ranger_bringup/`, root `README.md`) and were
   pulled in only because phase-1-description was originally branched
   from `humble`. Two cleanup options if they should be dropped:
   (a) interactive rebase to drop only those 9 commits while keeping
   `05e6f95` and `9d081fe`, or (b) cherry-pick the 2 phase-1 commits
   onto a fresh branch from `jazzy` and reset `phase-1-description` to
   that. Either is non-destructive at this point because nothing has
   been pushed. If they should stay, no action is needed.

2. **Is `solanrewaju2020@fau.edu` / `Shuaib Olanrewaju` the desired
   permanent maintainer for `ranger_mini_v3_description`?** I copied
   the active git identity per the round-02 instruction. Easy to
   change in a follow-up if the architect intends a project-level
   contact email instead.
