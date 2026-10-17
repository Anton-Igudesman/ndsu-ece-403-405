<script>
  import { onMount } from 'svelte'
  
  /** @type {{ mac: string, ip: string, auth_state: number }[]} */
  let clients = $state([])

  const AUTH_NONE = 0
  const AUTH_PENDING = 1
  const AUTH_CONTROLLER = 2
  const AUTH_ADMIN = 3

  // Convert the client authorization state into a readable label
  /** @param {number} authState */
  function getAuthLabel(authState) {
    switch (authState) {
      case AUTH_NONE:
        return 'Not Authorized'
      case AUTH_PENDING:
        return 'Pending Approval'
      case AUTH_CONTROLLER:
        return 'Controller'
      case AUTH_ADMIN:
        return 'Administrator'
      default:
        return 'Unknown'
    }
  }

  let loading = $state(true)
  let errorMessage = $state('')
  let actionErrorMessage = $state('')

  /** @type {WebSocket | null} */
  let adminSocket = null

  async function loadClients() {
    try {
      const response = await fetch('/api/admin/clients')

      if (!response.ok) {
        throw new Error()
      }

      clients = await response.json()
      errorMessage = ''
    } catch {
      errorMessage = 'Unable to retrieve connected clients'
    } finally {
      loading = false
    }
  }

  // Send a controller approval or denial to the ESP32
  /**
   * @param {string} action
   * @param {string} mac
   */
  function sendControllerAction(action, mac) {
    if (!adminSocket || adminSocket.readyState !== WebSocket.OPEN) {
      errorMessage = 'Administrator connection is unavailable'
      return
    }

    // Send the action and target client MAC through the existing WebSocket
    adminSocket.send(JSON.stringify({
      action,
      mac
    }))
  }

  /*
   Load the current client list when the dashboard opens.

   The WebSocket listens for access point client changes and immediately
   reloads the authoritative client list from the ESP32.
*/

  onMount(() => {
    loadClients()

    const protocol = 
      window.location.protocol === 'https:' ? 'wss:' : 'ws:'

    adminSocket = new WebSocket(
      `${protocol}//${window.location.host}/ws`
    )

    adminSocket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data)

        if (message.event === 'clients_changed') {
          loadClients()
        }

        if (message.event === 'controller_action_result') {
          if (message.success) {
            errorMessage = ''
            loadClients()
          } else {
            actionErrorMessage = `Controller action failed: ${message.error || 'Unknown error'}`
          }
        }
      } catch {
        console.error('Invalid WebSocket message received')
      }
    }

    adminSocket.onerror = () => {
      errorMessage = 'Dashboard live connection failed'
    }

    // Close the WebSocket when the dashboard is no longer active
    return () => {
      adminSocket?.close()
      adminSocket = null
    }
  })
</script>

<main class="min-h-screen bg-slate-950 text-white">
  <header class="border-b border-slate-800 bg-slate-900">
    <div class="mx-auto flex max-w-6xl items-center justify-between px-6 py-5">
      <div>
        <h1 class="text-2xl font-bold">
          LED Cube Admin
        </h1>

        <p class="mt-1 text-sm text-slate-400">
          Access Point Client Monitor
        </p>
      </div>

      <a
        href="#/"
        class="rounded-lg border border-slate-700 px-4 py-2 text-sm text-slate-300 hover:bg-slate-800"
      >
        Exit Admin
      </a>
    </div>
  </header>

  <section class="mx-auto max-w-6xl px-6 py-10">
    <div class="mb-6 flex items-end justify-between">
      <div>
        <h2 class="text-xl font-semibold">
          Connected Clients
        </h2>

        <p class="mt-1 text-sm text-slate-400">
          Devices connected to the LED-Cube access point.
        </p>
      </div>

      <div class="text-sm text-slate-400">
        {clients.length} connected
      </div>
    </div>

    <div class="overflow-hidden rounded-xl border border-slate-800 bg-slate-900">
      <table class="w-full text-left">
        <thead class="border-b border-slate-800">
          <tr class="text-sm text-slate-400">
            <th class="px-6 py-4 font-medium">
              MAC Address
            </th>

            <th class="px-6 py-4 font-medium">
              IP Address
            </th>

            <th class="px-6 py-4 font-medium">
              Status
            </th>
            <th class="px-6 py-4 text-left text-sm font-semibold text-slate-400">
              Actions
            </th>
          </tr>
        </thead>

        <tbody>
          {#each clients as client}
            <tr class="border-b border-slate-800 last:border-b-0">
              <td class="px-6 py-5 font-mono text-sm">
                {client.mac}
              </td>

              <td class="px-6 py-5 font-mono text-sm">
                {client.ip}
              </td>

              <td class="px-6 py-5">
                <span
                  class:text-yellow-400={client.auth_state === AUTH_PENDING}
                  class:text-green-400={client.auth_state === AUTH_CONTROLLER}
                  class:text-blue-400={client.auth_state === AUTH_ADMIN}
                  class:text-slate-400={client.auth_state === AUTH_NONE}
                  class="text-sm"
                >
                  {getAuthLabel(client.auth_state)}
                </span>
              </td>
              <td class="px-6 py-5">
                {#if client.auth_state === AUTH_PENDING}
                  <div class="flex gap-3">
                    <button
                      onclick={() => sendControllerAction('approve_controller', client.mac)}
                      class="rounded-lg bg-green-600 px-4 py-2 text-sm font-semibold text-white hover:bg-green-500"
                    >
                      Approve
                    </button>

                    <button
                      onclick={() => sendControllerAction('deny_controller', client.mac)}
                      class="rounded-lg bg-red-600 px-4 py-2 text-sm font-semibold text-white hover:bg-red-500"
                    >
                      Deny
                    </button>
                  </div>
                {/if}
              </td>
            </tr>
          {/each}

          {#if !loading && clients.length === 0 && !errorMessage}
            <tr>
              <td
                colspan="4"
                class="px-6 py-12 text-center text-sm text-slate-500"
              >
                No clients connected
              </td>
            </tr>
          {/if}
        </tbody>
      </table>

      {#if loading}
        <div class="px-6 py-12 text-center text-sm text-slate-500">
          Loading connected clients...
        </div>
      {/if}

      {#if errorMessage}
        <div class="border-t border-slate-800 px-6 py-4 text-sm text-red-400">
          {errorMessage}
        </div>
      {/if}
      {#if actionErrorMessage}
        <div class="border-t border-slate-800 px-6 py-4 text-sm text-red-400">
          {actionErrorMessage}
        </div>
      {/if}
    </div>
  </section>
</main>