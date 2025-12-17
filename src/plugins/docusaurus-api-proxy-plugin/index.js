// Docusaurus API Proxy Plugin
// This plugin adds proxy configuration for API requests

module.exports = function(context, options) {
  return {
    name: 'docusaurus-api-proxy-plugin',

    configureWebpack(config, isServer, utils) {
      return {
        devServer: {
          proxy: [
            {
              context: ['/api'],
              target: 'http://localhost:8000',
              changeOrigin: true,
              secure: false,
            },
          ],
        },
      };
    },
  };
};