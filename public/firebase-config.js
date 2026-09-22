// Firebase Configuration for SIH Swarm Monitoring Dashboard
import { initializeApp } from "https://www.gstatic.com/firebasejs/10.12.0/firebase-app.js";
import { getAnalytics } from "https://www.gstatic.com/firebasejs/10.12.0/firebase-analytics.js";

const firebaseConfig = {
  apiKey: "AIzaSyDUYKN62-pSwAv6FTqLzlDepg1sLzvxzQo",
  authDomain: "chat1-263fb.firebaseapp.com",
  projectId: "chat1-263fb",
  storageBucket: "chat1-263fb.firebasestorage.app",
  messagingSenderId: "272558417680",
  appId: "1:272558417680:web:b4f714e22300034e4ea795",
  measurementId: "G-LMDCNCX1BZ"
};

// Initialize Firebase
const app = initializeApp(firebaseConfig);
const analytics = getAnalytics(app);

export { app, analytics };
