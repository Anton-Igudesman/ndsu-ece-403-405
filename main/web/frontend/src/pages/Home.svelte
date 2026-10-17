<script>
  import { onMount } from 'svelte'

  const AUTH_NONE = 0
  const AUTH_PENDING = 1
  const AUTH_CONTROLLER = 2
  const AUTH_ADMIN = 3

  let authState = $state(AUTH_NONE)
  let requestError = $state('')
  let statusLoaded = $state(false)
  let requestSubmitting = $state(false)
  let statusChecking = false
  let statusVersion = 0

  // Submit a controller access request to the ESP32
  async function requestAccess() {
    if (requestSubmitting) return

    // Invalidate any status response started before this request
    statusVersion++

    requestSubmitting = true
    requestError = ''

    try {
      const response = await fetch('/api/controller/request', {
        method: 'POST'
      })

      if (!response.ok) {
        throw new Error(`Request failed: HTTP ${response.status}`)
      }

      const result = await response.json()

      if (result.status === 'pending') {
        authState = AUTH_PENDING
      }
    } catch (error) {
      requestError = error instanceof Error
        ? error.message
        : 'Unable to request controller access'
    } finally {
      requestSubmitting = false
    }
  }

  // Retrieve the current controller authorization state
  async function checkControllerStatus() {

    if (statusChecking || requestSubmitting) return
    statusChecking = true
    const currentVersion = statusVersion

    try {
      const response = await fetch('/api/controller/status')

      if (!response.ok) {
        throw new Error(`Status request failed: HTTP ${response.status}`)
      }

      const result = await response.json()

      // Ignore a status response if a newer access request has started
      if (currentVersion === statusVersion) {
        authState = result.auth_state
        requestError = ''
      }
    } catch (error) {
      // Ignore failures from status checks superseded by an access request
      if (currentVersion === statusVersion) {
        console.error('Unable to retrieve controller status:', error)
        requestError = 'Unable to retrieve controller status'
      }
    } finally {
      statusLoaded = true
      statusChecking = false
    }
  }

  // Check authorization when the page opens and every two seconds afterward
  onMount(() => {
    checkControllerStatus()

    const statusInterval = setInterval(() => {
      checkControllerStatus()
    }, 2000)

    // Stop checking when the user leaves the page
    return () => {
      clearInterval(statusInterval)
    }
  })
</script>

<main class="min-h-screen flex items-center justify-center bg-slate-950">
  <div class="text-center">
    <h1 class="text-5xl font-bold text-white">
      LED Cube
    </h1>

    <p class="mt-4 text-lg text-slate-400">
      Control Interface
    </p>

    {#if !statusLoaded}
      <p class="mt-8 text-lg text-slate-400">
        Checking authorization...
      </p>

    {:else if authState === AUTH_NONE}
      <button
        onclick={requestAccess}
        disabled={requestSubmitting}
        class="mt-8 rounded-lg bg-blue-600 px-6 py-3 font-semibold text-white hover:bg-blue-500 disabled:cursor-not-allowed disabled:opacity-50"
      >
        {requestSubmitting ? 'Submitting...' : 'Request Access'}
      </button>

    {:else if authState === AUTH_PENDING}
      <p class="mt-8 text-lg font-medium text-yellow-400">
        Awaiting Administrator Approval
      </p>

    {:else if authState === AUTH_CONTROLLER}
      <p class="mt-8 text-lg font-medium text-green-400">
        Controller Access Approved
      </p>

    {:else if authState === AUTH_ADMIN}
      <p class="mt-8 text-lg font-medium text-blue-400">
        Administrator Authenticated
      </p>
    {/if}
    {#if requestError}
      <p class="mt-4 text-sm text-red-400">
        {requestError}
      </p>
    {/if}
  </div>
</main>