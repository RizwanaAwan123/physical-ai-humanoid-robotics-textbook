// docusaurus-plugin-api-proxy/index.js
const path = require('path');

module.exports = function(context, options) {
  return {
    name: 'docusaurus-plugin-api-proxy',

    configureWebpack(config, isServer, utils) {
      return {
        devServer: {
          proxy: {
            '/api': {
              target: 'http://localhost:8000',
              changeOrigin: true,
              secure: false,
            },
          },
        },
      };
    },
  };
};