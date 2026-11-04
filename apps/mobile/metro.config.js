// Expo's default Metro config detects the pnpm workspace and watches the packages folder.
const { getDefaultConfig } = require('expo/metro-config');

const config = getDefaultConfig(__dirname);

module.exports = config;
