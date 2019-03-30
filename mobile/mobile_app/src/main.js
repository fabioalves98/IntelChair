import Vue from 'vue'
import App from './App.vue'
import router from './router'
import nipplejs from 'nipplejs';

Vue.config.productionTip = false

new Vue({
  router,
  nipplejs,
  render: h => h(App)
}).$mount('#app')
