# Prompt Documentation — AI Book & RAG Chatbot

## Book Generation Prompts
- *Chapter Generation:*  
  "Generate a detailed chapter on [topic] in academic textbook style, ready for Docusaurus integration."
- *Illustration Generation:*  
  "Create diagrams and visuals to support the chapter content."
- 

## RAG Chatbot Prompts
- *Vector Indexing:*  
  "Embed chapter content into vectors and store in Qdrant for RAG retrieval."
- *Query Handling:*  
  "Answer user questions using retrieved chunks. Include correct citations."
- *Highlight Override:*  
  "If text is highlighted, answer using only that selection."
- *Validation Prompt:*  
  "Check answers for accuracy, relevance, and citation alignment with chapters."