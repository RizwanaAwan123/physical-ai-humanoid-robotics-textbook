# Deployment Instructions

## Current Deployment
Your frontend is currently deployed at: https://physical-ai-humanoid-robotics-textbook-ac16l2i0a.vercel.app

## Backend Deployment (Required for Chatbot)
1. Deploy your FastAPI backend to a service like Render, Railway, or Heroku
2. Update the vercel.json file with your actual backend URL

## Vercel Configuration
Your vercel.json file is set up to proxy API requests to your backend service. You need to:

1. Replace `YOUR_BACKEND_URL` with your actual deployed backend URL
2. Redeploy your frontend to Vercel after updating

## Steps to Complete Chatbot Functionality:

### 1. Deploy Backend First
Deploy the backend service (in the /backend directory) to a platform like:
- Render (https://render.com)
- Railway (https://railway.app)
- Heroku (https://heroku.com)

Example using Render:
1. Create a new Web Service on Render
2. Connect to your GitHub repository
3. Set the Root Directory to `/backend`
4. Use Python runtime
5. Add environment variables:
   - QDRANT_URL
   - QDRANT_API_KEY
   - NEON_DATABASE_URL
   - JWT_SECRET

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
Make sure to set the following environment variables in your deployed backend:
- QDRANT_URL (your Qdrant cloud URL)
- QDRANT_API_KEY (your Qdrant API key)
- NEON_DATABASE_URL (your Neon PostgreSQL connection string)
- JWT_SECRET (for authentication)

## Testing
After deployment:
1. Visit your frontend: https://physical-ai-humanoid-robotics-textbook-ac16l2i0a.vercel.app
2. Go to the Chat page
3. Try asking a question - it should connect to your backend