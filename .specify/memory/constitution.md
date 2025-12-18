<!--
Sync Impact Report:
Version change: Template → 1.0.0
Constitution Type: MAJOR - Initial constitution establishment for AI/Spec-Driven Book Creation Project
Added sections: All core principles defined for technical book development
Modified principles: N/A (initial version)
Removed sections: None
Templates requiring updates:
  ✅ plan-template.md - reviewed, aligned with constitution principles
  ✅ spec-template.md - reviewed, aligned with constitution principles
  ✅ tasks-template.md - reviewed, aligned with constitution principles
  ✅ CLAUDE.md - project-specific agent guidance file exists and references constitution
Follow-up TODOs: None - all placeholders filled
-->

# AI-Driven Technical Book Constitution

## Core Principles

### I. Technical Accuracy (NON-NEGOTIABLE)

All explanations of tools, workflows, and frameworks MUST be technically correct and verifiable.

- Every command, configuration, and code sample MUST be tested and verified before inclusion
- All technical claims MUST reference official documentation or authoritative sources
- AI-generated content MUST be reviewed for correctness before acceptance
- Technical errors discovered MUST be corrected immediately with validation

**Rationale**: Incorrect technical content undermines the book's credibility and can mislead learners, causing frustration and wasted time.

### II. Clarity for Learners

Content MUST be accessible to beginner and intermediate-level readers with clear, structured explanations.

- Use simple language; avoid unnecessary jargon without definition
- Provide context before diving into complex concepts
- Structure content with clear headings, progressive disclosure, and logical flow
- Include concrete examples for abstract concepts
- Define terms on first use

**Rationale**: The book targets learners who need step-by-step guidance. Dense or unclear writing creates barriers to understanding.

### III. Consistency with Standards

All content MUST follow Docusaurus documentation standards and established conventions.

- Adhere to Docusaurus Markdown conventions (MDX format)
- Use consistent formatting for commands (code blocks), file paths (inline code), and notes (admonitions)
- Follow Docusaurus front matter requirements for metadata
- Maintain consistent terminology throughout the book
- Use Docusaurus components (tabs, code blocks, admonitions) appropriately

**Rationale**: Consistency reduces cognitive load for readers and ensures the book functions correctly as a Docusaurus site.

### IV. Practicality with Actionable Instructions

Every procedure MUST provide step-by-step instructions that readers can execute.

- Include all prerequisite steps explicitly
- Provide complete commands with necessary flags and arguments
- Show expected output or results for verification
- Include file paths and directory structures
- Explain what each step accomplishes

**Rationale**: Learners need actionable guidance, not just theoretical explanations. Incomplete instructions lead to frustration and abandonment.

### V. Reproducibility (NON-NEGOTIABLE)

All instructions MUST be executable as written by a reader following along.

- Commands MUST work in the specified environment
- File paths MUST be accurate and consistent
- Configuration files MUST be complete and valid
- Installation steps MUST include versions and dependencies
- Troubleshooting guidance MUST address common errors

**Rationale**: A technical book's value lies in its ability to guide readers to successful outcomes. Non-reproducible instructions destroy trust.

### VI. Verification Before Inclusion

All content MUST be validated before final inclusion in the book.

- Run all commands and verify output
- Test all configuration files for syntax and functionality
- Validate the Docusaurus build process completes without errors
- Check all internal and external links
- Review for ambiguous instructions that require clarification

**Rationale**: Prevention of errors is more efficient than correction. Validated content reduces reader support burden and increases book quality.

### VII. Educational Structure

Content MUST be organized to support progressive learning from fundamentals to advanced topics.

- Chapter order MUST follow logical learning progression
- Each chapter MUST build on previous chapters' concepts
- Include learning objectives at chapter start
- Provide summaries and next steps at chapter end
- Cross-reference related content appropriately

**Rationale**: Structured learning paths help readers build knowledge systematically, reducing overwhelm and improving retention.

## Content Requirements

### Mandatory Coverage

The book MUST include comprehensive coverage of:

1. **Spec-Kit Plus**: Usage patterns, workflows, templates, commands, and integration with development processes
2. **Docusaurus**: Setup, configuration, theming, content creation, plugins, and deployment to GitHub Pages
3. **Claude/Gemini Integration**: How AI tools are used for content generation, improvement, and validation
4. **Code Samples**: Complete, working examples with explanations
5. **Folder Structures**: Visual representations of project organization
6. **CLI Commands**: Full command syntax with descriptions and examples
7. **Configuration Files**: Complete, valid configuration examples with annotations
8. **Troubleshooting**: Common issues, error messages, and resolution steps
9. **Best Practices**: Recommended approaches based on real-world usage

### Content Validation Standards

Before any content is finalized:

- Configuration files MUST be syntactically valid (JSON, YAML, JavaScript as applicable)
- Commands MUST execute successfully in the documented environment
- Links MUST resolve correctly (internal navigation, external references)
- Code samples MUST be complete and runnable (not fragments missing context)

## Writing Standards

### Tone and Style

- **Instructive**: Direct, clear guidance on what to do and why
- **Beginner-friendly**: Assume minimal prior knowledge; explain concepts before using them
- **Professional**: Technical but accessible; avoid casual slang or overly formal academic language
- **Encouraging**: Support learners through challenges without condescension

### Formatting Consistency

All content MUST follow these formatting rules:

- **Commands**: Use fenced code blocks with language identifiers (```bash, ```javascript, etc.)
- **File paths**: Use inline code formatting (`path/to/file`)
- **Configuration**: Use fenced code blocks with appropriate language tags
- **Notes/Warnings**: Use Docusaurus admonitions (:::note, :::warning, :::tip)
- **Steps**: Use ordered lists for sequential procedures
- **Options**: Use unordered lists for non-sequential choices

### Originality

- All explanations MUST be original content, not plagiarized from other sources
- Paraphrasing technical documentation is permitted with attribution
- Code samples from official docs MUST be attributed with links
- AI-generated drafts MUST be reviewed, edited, and validated for accuracy and originality

## Project Constraints

### Scope Requirements

- **Chapter count**: Minimum 8 to 12 chapters covering all mandatory topics
- **Build requirement**: The Docusaurus site MUST build without errors (`npm run build`)
- **Deployment requirement**: The site MUST deploy successfully to GitHub Pages
- **Navigation**: Clear chapter progression with working internal links
- **Markdown compliance**: All files MUST be valid Docusaurus-compatible MDX

### Technical Validation Gates

Before considering the book complete, the following MUST pass:

1. `npm run build` executes without errors
2. `npm run serve` displays the site correctly
3. Deployment to GitHub Pages succeeds
4. All commands in the book have been executed and verified
5. All configuration files validate successfully
6. All links (internal and external) resolve correctly
7. Navigation structure is logical and complete

## Success Criteria

The book is considered successful when:

1. **Build Success**: Docusaurus builds the site without errors or warnings
2. **Deployment Success**: The site deploys to GitHub Pages and displays correctly
3. **Reproducibility**: A new user can follow all instructions and achieve documented outcomes
4. **Accuracy**: No incorrect commands, configurations, or technical claims remain
5. **Navigation**: Clear, consistent layout with logical chapter flow
6. **Usability**: The book functions as effective learning resource for target audience

### Definition of Done for Each Chapter

A chapter is complete when:

- All commands have been tested and verified
- All configuration files are valid and complete
- All code samples are executable and correct
- All links are functional
- Content follows writing standards and formatting consistency
- Technical accuracy has been validated
- The chapter builds without errors in Docusaurus
- Prerequisites and learning objectives are clearly stated

## Governance

### Amendment Process

This constitution may be amended when project requirements evolve:

1. Proposed amendments MUST be documented with rationale
2. Version number MUST be incremented following semantic versioning:
   - **MAJOR**: Backward-incompatible principle changes or removals
   - **MINOR**: New principles added or material expansions
   - **PATCH**: Clarifications, wording improvements, non-semantic fixes
3. All dependent templates (spec, plan, tasks) MUST be reviewed for consistency
4. Amendments MUST be approved before taking effect

### Compliance Review

All work MUST be verified against this constitution:

- Specifications MUST reference relevant principles
- Plans MUST include constitution check gates
- Task lists MUST enforce principle compliance
- Code reviews MUST verify adherence to standards
- Build failures MUST be addressed before proceeding

### Enforcement

Non-compliance with NON-NEGOTIABLE principles (Technical Accuracy, Reproducibility) requires immediate remediation. Other principles should be enforced through review processes with explicit justification if deviation is necessary.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
