# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### I. Technical Excellence & Depth
Content must maintain the highest standards of technical accuracy and depth. All concepts, algorithms, and implementations must be thoroughly researched and validated by domain experts. Code examples must be executable, well-tested, and follow industry best practices for robotics and AI development.

### II. Modular Structure Integrity (NON-NEGOTIABLE)
The book structure follows a fixed 4-module, 4-chapter architecture: Module 1 (ROS 2), Module 2 (Digital Twin), Module 3 (AI-Robot Brain), and Module 4 (Vision-Language-Action). Each module contains exactly 4 detailed chapters, with the capstone project integrating all modules. This structure is immutable without formal amendment process.

### III. AI-Native Development & Spec-Driven Approach
All content creation must follow Spec-Kit Plus methodology: specification → clarification → plan → tasks → implementation → validation. This ensures systematic, predictable development aligned with hackathon requirements and professional engineering standards.

### IV. Professional UI/UX Standards
The book interface must embody professional design standards with custom Docusaurus theming, responsive layout, accessible navigation, syntax highlighting, mathematical notation rendering, and intuitive user experience that matches premium educational platforms.

### V. Free-Tier Infrastructure Optimization
The RAG system and backend infrastructure must operate effectively within free-tier limits: Qdrant Cloud free tier for vector storage, Neon Serverless Postgres for metadata, and efficient API usage patterns that minimize costs while maintaining functionality.

### VI. RAG System Content Fidelity
The RAG chatbot must answer exclusively from book content. Responses must be grounded in retrieved context with clear source attribution. Any query outside the book's scope must be explicitly acknowledged with "This information is not available in the book."

## Content Standards

All chapters must include six mandatory sections:
1. **Learning Objectives**: 3-5 measurable outcomes with Bloom's taxonomy levels
2. **Theoretical Foundations**: Mathematical formulations and conceptual explanations
3. **Hands-On Implementation**: Executable code examples with detailed walkthroughs
4. **Practical Examples**: Real-world scenarios demonstrating concepts
5. **Exercises & Challenges**: Progressive difficulty (beginner, intermediate, advanced)
6. **Further Reading**: Curated references to documentation, research papers, and tutorials

## Technical Architecture

### Frontend Stack
- Docusaurus 3.x with custom theme and MDX support
- KaTeX for mathematical notation
- Mermaid for diagram rendering
- Prism.js for syntax highlighting
- Responsive CSS with custom color palette

### Backend Stack
- FastAPI (Python 3.10+) for API services
- Qdrant for vector embeddings and semantic search
- Neon Postgres for conversation history and metadata
- OpenAI APIs for embeddings and generation
- Better-Auth for user authentication

### Development Workflow
- Git-based version control with feature branches
- Spec-Driven Development (SDD) methodology
- Continuous integration with automated testing
- Quality gates with linting and validation
- Semantic versioning (MAJOR.MINOR.PATCH)

## Governance

### Amendment Process
Changes to this constitution require formal ADR (Architectural Decision Record) with justification, alternatives analysis, and stakeholder approval.

### Versioning Policy
Semantic versioning with backward compatibility maintained within minor versions.

### Compliance
All development activities must demonstrate compliance with stated principles through PHRs (Prompt History Records) and quality gates.

**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06