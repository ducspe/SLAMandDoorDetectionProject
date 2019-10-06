<template>
  <q-card class="doorList">
    <q-badge v-if="online" color="positive" floating>ONLINE</q-badge>
    <q-badge v-else color="negative" floating>OFFLINE</q-badge>
    <!-- <q-inner-loading :showing="loading"></q-inner-loading> -->
    <q-table
      class="door-table"
      title="Detected Doors"
      :data="data"
      :columns="columns"
      :loading="loading"
      row-key="name"
      flat
      bordered
    ></q-table>
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
  name: 'DoorList',
  beforeMount () {
    this.reload()
    this.getDoorList()
  },
  data () {
    return {
      loading: false,
      online: false,
      columns: [
        {
          name: 'doorId',
          required: true,
          label: 'Door ID',
          align: 'right',
          field: 'doorId',
          sortable: true
        },
        {
          name: 'status',
          label: 'Status',
          field: 'status',
          align: 'left',
          sortable: true
        }
      ],
      data: []
    }
  },
  methods: {
    getDoorList () {
      console.log('getDoorList')

      this.loading = true
      axios.get('http://localhost:5000/doors/list')
        .then(response => {
          console.log(response)
          this.data = response.data['doorList']
          this.loading = false
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
        this.reload()
        this.getDoorList()
      }.bind(this), 1000)
    }
  }
}
</script>
