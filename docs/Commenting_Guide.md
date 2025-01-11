# Commenting Guide

The goal of comments are to make it easy for someone else to use your code and for someone who hasn't seen your code but knows how to program a robot to figure out how your code works

- Comments only go in the `.h` files
- Don't comment self-explanatory code
  - `OI.h`
  - `Initialize`, `Execute`, `End`, `IsFinished` functions

> [!NOTE]  
> When making a comment, if you want the `@param` and `@return` tags to work, you will need a second asterisk at the beginning of the comment, as seen in the example images below

## Subsystems
Brief description of what part of the robot it controls and high level
description of the public functions
![Subsystems](/docs/images/commenting_launchersubsystem.png)

## Commands
What it does and when it should be called

![Subsystems](/docs/images/commenting_launchercommand.png)


## Functions
What it does, parameters, returns, and any nuances for how it works

If the function controls a motor or piston, say which direction is positive

## State Machines
What each state does and when it should be active