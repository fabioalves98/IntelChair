<template>
    <div>
        <nav>
          <router-link v-for="routes in links"
          v-bind:key="routes.id" :to="`${routes.page}`">{{routes.text}}</router-link>  
        </nav>    
    </div>
</template>

<script>
export default {
    name:'Navigation',
    data(){
        return{
            links: [
                {
                    id: 0,
                    text: 'Manual Mode',
                    page: '/'
                },
                {
                    id: 1,
                    text: 'Map',
                    page:'/Map'
                }
            ]
        }

    }
}
</script>

<style scoped>
nav{
    display:inline-block;
    margin:60px auto 45px;
    background-color:#5597b4;
    box-shadow:0 1px 1px #ccc;
    border-radius:2px;
}

nav a{
    display:inline-block;
    padding: 18px 30px;
    color:#fff !important;
    font-weight:bold;
    font-size:16px;
    text-decoration:none !important;
    line-height:1;
    text-transform: uppercase;
    background-color:transparent;

    -webkit-transition:background-color 0.25s;
    -moz-transition:background-color 0.25s;
    transition:background-color 0.25s;
}

nav a:first-child{
    border-radius:2px 0 0 2px;
}

nav a:last-child{
    border-radius:0 2px 2px 0;
}

.router-link-active{
   background-color:#e35885; 

}

</style>


