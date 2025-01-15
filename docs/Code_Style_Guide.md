# Code Style Guide
This page documents good practices for how to format and style our C++ code

## Cases
- File names are in Pascal Case (`PascalCase`)
- Function names are in Pascal Case (`PascalCase`)
- Classes are in Pascal Case (`PascalCase`)
- Constants should be in capitalized snake case (`CAPITALIZED_SNAKE_CASE`)
- Variables are in lower snake case (`lower_snake_case`)
- Namespaces are Camel Case and one word (`Namespace`)

## Formatting
- Indentation is 4 spaces
- Private variables and functions begin with an underscore (`_lower_snake_case`)
- Comments should have a space between the double forward slashes (`// This is a comment`)
- Class and function contents' corresponding curly brackets should have a space before them
    ```cpp
    class Driver_Interface {
        -- snip --
    };

    void DrivetrainSubsystem::Periodic() {
        -- snip --
    }
    ```
- Functions should never be condensed to one line

## File Names
- Corresponding `.cpp` and `.h` files should have the same name
- Subsystem files are named `<subsystem name>Subsystem`
- Commands are separated into `/auton` and `/teleop` folders
  - The names should be either `Auton<command name>` or `Teleop<command name>`

## Other
- Namespaces shouldn't be used in `.h` files, but they can be used in `.cpp` files
- Don't use `@pragma once`, instead use `#ifndef <capitalized filename and file extension>`
   ```cpp
    #ifndef FILENAME_H
    #define FILENAME_H
 
    -- snip --

    #endif
    ```
