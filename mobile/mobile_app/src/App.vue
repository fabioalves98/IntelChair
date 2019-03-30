<template>
  <div id="app">
    <StatusBar></StatusBar>
    <router-view/>
    <Navigation></Navigation> 
  </div>
</template>

<script>
import Navigation from './components/nav.vue'
import StatusBar from './components/statusbar.vue'
export default {
  name: 'app',
  components: {
    'Navigation': Navigation,
    'StatusBar': StatusBar,
  }
}
</script>


<style>
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #000000;
  margin-top: 60px;
}
</style>
