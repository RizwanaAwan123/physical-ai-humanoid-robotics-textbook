# My Book Project Specifications

## Project Overview

This project is an AI-native textbook website focused on Physical AI and Humanoid Robotics. It features a modern Docusaurus UI with a RAG (Retrieval Augmented Generation) chatbot that allows users to ask questions about the textbook content.

## Core Features

### 1. Educational Content Platform
- **Purpose**: Deliver comprehensive educational content about Physical AI and Humanoid Robotics
- **Structure**: Organized into 6 main chapters with consistent formatting
- **Format**: Clean, readable Docusaurus-based interface optimized for learning

### 2. RAG Chatbot Integration
- **Purpose**: Allow learners to ask questions about the textbook content and receive accurate answers
- **Features**: Source citations, contextual understanding, and relevant information retrieval
- **Technology**: Vector search using Qdrant with semantic similarity matching

### 3. Responsive Design
- **Purpose**: Ensure content is accessible across all devices (desktop, tablet, mobile)
- **Standards**: WCAG 2.1 AA compliance for accessibility
- **Performance**: Fast loading times with Lighthouse scores >90

## Technical Architecture

### Frontend
- **Framework**: Docusaurus 3.x with React 18+
- **Deployment**: GitHub Pages for static hosting
- **Features**: Clean, modern UI with navigation sidebar, responsive layout

### Backend API
- **Framework**: FastAPI (Python 3.11+)
- **Purpose**: Handle RAG queries and chatbot interactions
- **Deployment**: Railway or Render (free tier)

### Data Storage
- **Vector Database**: Qdrant (free tier) for embedding storage
- **Metadata**: Neon PostgreSQL (free tier) for chunk metadata
- **Content**: Markdown files for textbook content

## User Experience

### Primary User Stories

1. **Reading Chapters** - Learners can navigate through well-structured chapters with consistent formatting
2. **Asking Questions** - Learners can ask questions about content via the integrated chatbot
3. **Responsive Access** - Content works seamlessly across different devices
4. **Quick Navigation** - Easy navigation between chapters with instant load times

## Development Constraints

### Technical Requirements
- Use TypeScript for type safety
- Implement rate limiting (10 requests/minute per IP)
- Support free-tier deployment options only
- Ensure performance targets (load times <2s)

### Content Requirements
- Exactly 6 chapters of 10-25 pages each
- Consistent structure (Learning Objectives, Core Concepts, Practical Applications, Summary)
- Markdown-based content with proper formatting

## Success Metrics

### Performance
- Page load time <2 seconds
- Chatbot response time <2 seconds
- Lighthouse score >90 across all pages

### User Experience
- Mobile-friendly and accessible
- All 6 chapters complete and well-structured
- RAG chatbot provides accurate answers with proper citations

## Deployment

### Frontend Deployment
- GitHub Pages with single command deployment
- Optimized build process (<3 minutes)

### Backend Deployment
- Free-tier hosting on Railway or Render
- Health checks and monitoring implemented