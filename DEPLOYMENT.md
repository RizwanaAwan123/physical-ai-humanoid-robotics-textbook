# Deployment Instructions

## Current Deployment
Your frontend is currently deployed at: https://physical-ai-humanoid-robotics-textbook-ac16l2i0a.vercel.app

## Complete Deployment for Full Functionality

### 1. Backend Deployment (Required for Chatbot)
Deploy your FastAPI backend to a service like Render, Railway, or Heroku:

**Using Render (Recommended):**
1. Go to [https://render.com](https://render.com)
2. Sign in and click "New +" → "Web Service"
3. Connect to your GitHub repository
4. Select `RizwanaAwan123/physical-ai-humanoid-robotics-textbook`
5. Set Root Directory: `/backend`
6. Environment: `Python`
7. Build Command: `pip install -r requirements.txt`
8. Start Command: `uvicorn main:app --host 0.0.0.0 --port $PORT`
9. Add Environment Variables (get these from your `backend/.env.example`):
   - QDRANT_URL: Your Qdrant cloud URL
   - QDRANT_API_KEY: Your Qdrant API key
   - NEON_DATABASE_URL: Your Neon PostgreSQL connection string
   - JWT_SECRET: A random secret string

**Important:** Never commit actual credentials to git. Use the example values from `backend/.env.example` as reference.

### 2. Update vercel.json
After deploying your backend, update the vercel.json file:

```json
{
  "version": 2,
  "builds": [
    {
      "src": "package.json",
      "use": "@vercel/static-build",
      "config": {
        "distDir": "build"
      }
    }
  ],
  "rewrites": [
    {
      "source": "/api/(.*)",
      "destination": "YOUR_BACKEND_URL/api/$1"
    }
  ]
}
```

Replace `YOUR_BACKEND_URL` with the actual URL of your deployed backend (e.g., `https://your-app.onrender.com`).

### 3. Redeploy Frontend to Vercel
1. Commit and push the updated vercel.json file
2. Vercel will automatically redeploy your site
3. The chatbot should now work with your deployed backend

## Environment Variables for Backend
Set these environment variables in your deployed backend service:
- QDRANT_URL (your Qdrant cloud URL)
- QDRANT_API_KEY (your Qdrant API key)
- NEON_DATABASE_URL (your Neon PostgreSQL connection string)
- JWT_SECRET (for authentication)

## Security Notes
- The `.env` files are excluded from git to prevent credential exposure
- Use `.env.example` files to show required variables without exposing secrets
- Never share your actual API keys or database URLs publicly

## Testing
After complete deployment:
1. Visit your frontend: https://physical-ai-humanoid-robotics-textbook-ac16l2i0a.vercel.app
2. Navigate to the Chat page
3. Try asking a question - it should connect to your backend and respond based on your textbook content
4. The book content should display properly as it's static content

## Troubleshooting
- If the book content doesn't load properly, check the baseUrl configuration in docusaurus.config.ts
- If the chatbot doesn't work, verify that your backend is deployed and accessible
- Check browser console for any error messages