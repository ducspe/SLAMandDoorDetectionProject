<template>
  <q-card class="map">
    <q-badge v-if="online" color="positive" floating>ONLINE</q-badge>
    <q-badge v-else color="negative" floating>OFFLINE</q-badge>
    <q-inner-loading :showing="loadMapImage"></q-inner-loading>
    <img :src="mapImage" />
    <q-card-section>
        <div class="text-h6">
          Map image
        </div>
    </q-card-section>
  </q-card>
</template>

<style lang="stylus" scoped>
.q-badge
  z-index: 9999999
.map img
  width: auto
  height: 100%
  max-height: 700px
</style>

<script>
import axios from 'axios'

export default {
  name: 'Map',
  beforeMount () {
    this.getMapImage()
    this.reload()
  },
  data () {
    return {
      mapImage: '',
      loadMapImage: true,
      online: false
    }
  },
  methods: {
    getMapImage () {
      console.log('getMapImage')

      this.loadMapImage = true
      axios.get('http://localhost:5000/map/')
        .then(response => {
          console.log(response)
          this.mapImage = response.data['mapImage']
          this.loadMapImage = false
          this.online = true
        })
        .catch(e => {
          this.errors.push(e)
          this.loadMod = false
          this.online = false
        })
    },
    reload () {
      setTimeout(function () {
        this.getMapImage()
        this.reload()
      }.bind(this), 1000)
    }
  }
}
</script>
