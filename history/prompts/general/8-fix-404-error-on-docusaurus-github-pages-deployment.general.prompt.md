---
id: 8
title: Fix 404 error on Docusaurus GitHub Pages deployment
stage: general
date: 2025-12-07
surface: agent
model: gemini-1.5-flash-latest
feature: none
branch: main
user: unknown
command: is url pe 404 ka error aa raha hai is ko check kro or fix kro https://umar-30.github.io/Physical-AI-Humanoid-Book/
labels: ["fix", "docusaurus", "github-pages", "404"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - roboticAI_book/docusaurus.config.ts
tests:

---

## Prompt

https://umar-30.github.io/Physical-AI-Humanoid-Book/is url pe 404 ka error aa raha hai is ko check kro or fix kro https://umar-30.github.io/Physical-AI-Humanoid-Book/

## Response snapshot

Investigated the Docusaurus configuration and GitHub Actions workflow.
Identified incorrect `url`, `baseUrl`, `organizationName`, and `projectName` in `roboticAI_book/docusaurus.config.ts`.
Updated `docusaurus.config.ts` to reflect the correct GitHub Pages deployment paths.
Verified `roboticAI_book/.github/workflows/deploy.yml` is correctly configured to build and deploy the Docusaurus project.

## Outcome

- ✅ Impact: Identified and applied the configuration fix for the 404 error on the Docusaurus GitHub Pages deployment. The site should now be accessible after a new deployment.
- 🧪 Tests: The fix requires a new GitHub Pages deployment to be fully verified by accessing the URL.
- 📁 Files: Modified `roboticAI_book/docusaurus.config.ts`.
- 🔁 Next prompts: Instruct the user to commit and push changes to trigger a new deployment.
- 🧠 Reflection: The 404 error was caused by incorrect `baseUrl` and related parameters in `docusaurus.config.ts`. The GitHub Actions workflow `deploy.yml` was found to be correctly set up, but relied on the Docusaurus configuration for correct path generation. The fix involves updating `docusaurus.config.ts` to match the GitHub Pages URL structure.

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
