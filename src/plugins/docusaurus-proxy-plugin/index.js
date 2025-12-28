module.exports = function (context, options) {
  return {
    name: 'docusaurus-proxy-plugin',
    configureWebpack(config, isServer, utils) {
      return {
        devServer: {
          proxy: [
            {
              context: ['/api'],
              target: 'http://localhost:8000',
              changeOrigin: true,
              pathRewrite: { '^/api': '/api' },
            },
          ],
        },
      };
    },
  };
};