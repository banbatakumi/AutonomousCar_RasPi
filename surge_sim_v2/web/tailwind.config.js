/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{js,jsx}"],
  theme: {
    extend: {
      colors: {
        surge: {
          bg: "#121216",
          panel: "#1e1e26",
          accent: "#3c82f0",
          ok: "#34d399",
          warn: "#fbbf24",
          danger: "#ef4444",
        },
      },
    },
  },
  plugins: [],
};
