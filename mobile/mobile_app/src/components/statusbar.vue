<template>   
    <ul>
        <li v-for="statusText in textData" :value="statusText.id" :key="statusText.id">
            {{ statusText.text }}
        </li>
    </ul>
</template>

<script>
export default {
    name:'StatuBar',
    data(){
        return{
            textData: [
                {
                    id: 0,
                    text: 'Status',
                },
                {
                    id: 1,
                    text: 'Battery',
                },
                                {
                    id: 2,
                    text: 'VelocityLevel',
                }
            ]
        }

    }
}
</script>

<style scoped>
li{
    display:inline-block;
    margin:60px auto 45px;
    background-color:#5597b4 !important;
    box-shadow:0 1px 1px #ccc;
    border-radius:2px;
    padding: 18px 30px;
    color:#fff !important;
    font-size:16px;
    text-decoration:none !important;
    line-height:1;
    text-transform: uppercase;
    background-color:transparent;

    -webkit-transition:background-color 0.25s;
    -moz-transition:background-color 0.25s;
    transition:background-color 0.25s;
}
</style>
