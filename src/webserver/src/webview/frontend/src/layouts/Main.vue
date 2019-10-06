<template>
  <q-layout view="lHh lPr lFf">
    <q-header elevated>
      <q-toolbar>

        <q-toolbar-title>
          Ben Stiller Door Detector
        </q-toolbar-title>
      </q-toolbar>
    </q-header>

    <q-page-container>
      <router-view />
    </q-page-container>
  </q-layout>
</template>

<script>
import { openURL } from 'quasar'

export default {
  name: 'BenStillerMain',
  methods: {
    openURL
  }
}
</script>

<style>
</style>
