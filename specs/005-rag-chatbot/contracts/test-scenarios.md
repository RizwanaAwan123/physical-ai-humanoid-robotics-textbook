# Test Scenarios: RAG Chatbot API

**Feature**: 005-rag-chatbot | **Date**: 2025-12-10

## Scenario 1: Simple Question
**Query**: "What is ROS 2?"
**Expected**: Response with definition from Module 1, citations, <3s latency

## Scenario 2: Text Selection
**Query**: "Explain 'URDF'" with `selected_text="URDF"`
**Expected**: Contextual explanation, Module 1 citations

## Scenario 3: Follow-up
**Q1**: "What is Isaac Sim?" ’ **Q2**: "How do I install it?"
**Expected**: System understands "it"=Isaac Sim from context

## Scenario 4: Out-of-Scope
**Query**: "What is the capital of France?"
**Expected**: "I don't have information about this in the textbook"

## Scenario 5: Rate Limiting
**Action**: 20 queries in 1 minute
**Expected**: 429 error after 15th query, `Retry-After` header

## Performance Targets
- Response time: <3s (P95)
- Vector search: <500ms
- Concurrent users: 100
