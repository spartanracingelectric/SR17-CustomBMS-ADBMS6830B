# R&D Custom BMS(ADBMS6830B)

## Environment Setup / Tools Needed

To work on this project, install the following tools:

1. **STM32CubeIDE**
2. **BusMaster**
3. **Peak CAN Driver**
4. **ST-LINK/V2 Driver**
5. **A Windows Laptop**
6. **Git**

Ensure all tools are properly set up before contributing.

## Resources

1. [Slave Battery Moniter IC LTC6811](https://www.analog.com/media/en/technical-documentation/data-sheets/ltc6811-1-6811-2.pdf)
2. [Master Chip STM32F105](https://www.st.com/resource/en/datasheet/stm32f105r8.pdf)
3. [Git Cheat Sheet](https://education.github.com/git-cheat-sheet-education.pdf)
4. [Eclipse Terminal](https://marketplace.eclipse.org/free-tagging/terminal#:~:text=by%20Martin%20Oberhuber-,A%20fully%20working%20command%2Dline%20Terminal%20inside%20Eclipse.,Previous%20sessions%20are...)

## Notes

Any updates to the DBF should also be reflected in the DBC. BusMaster has a convert tool for converting DBF to DBC.

## Coding Conventions

- Variables and functions should use `camelCase`
- Functions names should follow the structure of `Component_functionAction()`
  - ex: `CAN_sendSummary()`
- Docstrings should be included for each function
  - Details on parameter, return value, and description of function
  - Add information on units when necessary
    - ex: `millivolts`

## Pull Request (PR) Guidelines

Verify all features on hardware before making a PR.

### PR Titles
- PR titles should be a **one-sentence overall description** of the PR.
- If you can't fit an overall description in the PR title, the PR is likely too large and should be broken into smaller changes.

### PR Description
- The description should be more **in-depth**, explaining:
  - The purpose of the PR.
  - The changes made.
  - Any other relevant details. 

### PR Practices
- After completing testing for a feature, **squash commits** before making a PR.
- PRs should be **for a singular feature**.
- Avoid PRs with **giant changes**

### Approval Process
- PRs must be **approved by at least one designer** before merging, even for designers 😠. 

---

Shoutout to ChatGPT for generating this README.

