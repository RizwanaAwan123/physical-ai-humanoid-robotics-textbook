# Deployment Instructions

## Backend Deployment
1. Deploy your FastAPI backend to a service like Render, Railway, or Heroku
2. Update the vercel.json file with your actual backend URL

## Vercel Configuration
Your vercel.json file is set up to proxy API requests to your backend service. You need to:

1. Replace `https://your-fastapi-backend.onrender.com` with your actual deployed backend URL
2. The current vercel.json file is a template that needs to be updated

## Steps to Deploy:

### 1. Deploy Backend First
Deploy the backend service (in the /backend directory) to a platform like:
- Render (https://render.com)
- Railway (https://railway.app)
- Heroku (https://heroku.com)

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
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "YOUR_BACKEND_URL/api/$1",
      "headers": {
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET, POST, PUT, DELETE, OPTIONS",
        "Access-Control-Allow-Headers": "X-Requested-With, Content-Type, Accept"
      }
    }
  ],
  "rewrites": [
    {
      "source": "/api/:path*",
      "destination": "YOUR_BACKEND_URL/api/:path*"
    }
  ]
}
```

Replace `YOUR_BACKEND_URL` with the actual URL of your deployed backend.

### 3. Deploy Frontend to Vercel
After updating vercel.json, deploy your frontend to Vercel.

## Environment Variables
Make sure to set the following environment variables in your deployed backend:
- QDRANT_URL
- QDRANT_API_KEY
- NEON_DATABASE_URL
- JWT_SECRET