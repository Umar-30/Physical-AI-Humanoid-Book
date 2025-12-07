# Data Model for Chapter 1: Introduction to ROS 2

This document outlines the conceptual data model for the content of Chapter 1, based on the feature specification. It defines the key entities, their attributes, and relationships, guiding the structured creation of the chapter.

## Entities

### 1. Chapter

Represents the overarching structure and metadata of Chapter 1.

-   **Attributes**:
    -   `title`: String (e.g., "Introduction to ROS 2 & The Philosophy of Robotic Middleware")
    -   `number`: Integer (e.g., 1)
    -   `slug`: String (URL-friendly identifier)
    -   `word_count_target`: String (e.g., "1500-2000 words")
    -   `target_audience`: List of Strings (e.g., "University students", "Software developers")
    -   `success_criteria_ids`: List of Strings (References to SC-xxx from Feature Spec)
    -   `frontmatter`: YAML/JSON object (Docusaurus-specific metadata like `sidebar_position`)
    -   `sections`: List of Section entities (Composition)
    -   `learning_objectives`: List of LearningObjective entities (Composition)
    -   `key_takeaways`: List of KeyTakeaway entities (Composition)
    -   `glossary_terms`: List of GlossaryTerm entities (Composition)

### 2. Section

Represents a logical division within the chapter.

-   **Attributes**:
    -   `title`: String (e.g., "Why Robotic Middleware Matters")
    -   `header_level`: Integer (2 for H2, 3 for H3, etc.)
    -   `slug`: String
    -   `word_count_allocation`: String (e.g., "200-300 words")
    -   `content_blocks`: List of ContentBlock entities (Ordered composition of text, diagrams, code, questions)

### 3. LearningObjective

Specific, measurable outcomes for the reader.

-   **Attributes**:
    -   `id`: String (e.g., "LO-001")
    -   `description`: String
    -   `mapped_sections`: List of Section slugs (References to sections where objective is covered)

### 4. KeyTakeaway

Concise summaries of important points at the end of the chapter/sections.

-   **Attributes**:
    -   `id`: String (e.g., "KT-001")
    -   `description`: String
    -   `origin_section_slug`: Optional String (Section it summarizes)

### 5. ContentBlock (Abstract)

A base entity for different types of content within a Section.

-   **Type-specific Attributes**:
    -   **Text**: `body` (Markdown string)
    -   **Diagram**: `type`, `description`, `code` (for Mermaid), `alt_text`
    -   **CodeBlock**: `language`, `content`, `expected_output`, `verification_commands`
    -   **Question**: `text`, `expected_answer_keywords`, `type` (e.g., comprehension, multiple-choice)
    -   **Exercise**: `title`, `description`, `steps` (list of instructions), `verification_criteria`
    -   **Admonition**: `type` (note, tip, warning), `content` (Markdown string)

### 6. GlossaryTerm

Technical terms defined in the chapter.

-   **Attributes**:
    -   `term`: String (e.g., "DDS")
    -   `definition`: String
    -   `first_appearance_section_slug`: String

## Relationships

-   **Chapter HAS MANY Sections**: Chapter is composed of multiple Sections.
-   **Chapter HAS MANY LearningObjectives**: Chapter defines multiple learning objectives.
-   **Chapter HAS MANY KeyTakeaways**: Chapter concludes with key takeaways.
-   **Section CONTAINS MANY ContentBlocks**: Sections are composed of ordered content blocks.
-   **LearningObjective MAPS TO Sections**: Learning objectives are covered across various sections.
-   **ContentBlock REFERENCES GlossaryTerm**: Text content blocks may reference glossary terms.

## Data Flow (Conceptual)

1.  **Specification (`spec.md`)**: Defines overall requirements, user stories, success criteria.
2.  **Implementation Plan (`plan.md`)**: Outlines technical approach, research, design.
3.  **Research Findings (`research.md`)**: Resolves unknowns, informs decisions.
4.  **Data Model (`data-model.md`)**: Structures the content entities.
5.  **Actual Chapter Content (Markdown files)**: Realization of the data model.
6.  **Docusaurus Build**: Processes Markdown and associated assets into a website.
