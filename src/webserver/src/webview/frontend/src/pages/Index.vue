<template>
  <q-page :padding="true">
    <div class="q-pa-md">
      <div class="row q-col-gutter-md" :padding="true">
        <div class="col">
          <Map />
        </div>
        <div class="col-4">
          <Detection />
          <DoorList />
        </div>
      </div>
    </div>
  </q-page>
</template>

<style lang="stylus" scoped>
.row
  width: 100%
  height: 100%

.door-detection
  width: 100%
.q-card
  margin: 15px 0
</style>

<script>
import Map from '../components/Map'
import Detection from '../components/Detection'
import DoorList from '../components/DoorList'

export default {
  name: 'PageIndex',
  components: {
    Map,
    Detection,
    DoorList
  }
}
</script>
