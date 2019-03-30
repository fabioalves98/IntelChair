import Vue from 'vue'
import Router from 'vue-router'
import Manual from './views/Manual.vue'
import Map from './views/Maps.vue'

Vue.use(Router)

export default new Router({
  mode: 'history',
  base: process.env.BASE_URL,
  routes: [
    {
      path: '/',
      name: 'manual',
      component: Manual
    },
    {
      path: '/map',
      name: 'map',
      component: Map
    }
  ]
})
