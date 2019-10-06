<template>
  <q-card class="door-detection">
    <q-badge v-if="online" color="positive" floating>ONLINE</q-badge>
    <q-badge v-else color="negative" floating>OFFLINE</q-badge>
    <q-inner-loading :showing="loadDetectionImage"></q-inner-loading>
    <img :src="detectionImage" />
    <q-card-section>
        <div class="text-h6">
          Detection image
        </div>
    </q-card-section>
  </q-card>
</template>

<style lang="stylus" scoped>
.q-badge
  z-index: 9999999
</style>

<script>
import axios from 'axios'

export default {
  name: 'Door-Detection',
  beforeMount () {
    this.getDetectionImage()
    this.reload()
  },
  data () {
    return {
      detectionImage: '',
      loadDetectionImage: true,
      online: false
    }
  },
  methods: {
    getDetectionImage () {
      console.log('getDetectionImage')

      this.loadDetectionImage = true
      axios.get('http://localhost:5000/detection/')
        .then(response => {
          this.detectionImage = response.data['detectionImage']
          this.loadDetectionImage = false
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
        this.getDetectionImage()
        this.reload()
      }.bind(this), 1000)
    }
  }
}
</script>
