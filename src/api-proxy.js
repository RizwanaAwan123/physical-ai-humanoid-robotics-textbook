// This is a client-side module to handle API calls
// It will be used by the chatbot component

export const apiCall = async (endpoint, data) => {
  try {
    // In development, this will be proxied to localhost:8000
    // In production, this will need to be configured appropriately
    const response = await fetch(`/api${endpoint}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(data),
    });

    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }

    return await response.json();
  } catch (error) {
    console.error('API call error:', error);
    throw error;
  }
};