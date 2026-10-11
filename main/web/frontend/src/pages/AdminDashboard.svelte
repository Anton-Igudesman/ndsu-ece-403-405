<script>
  import { onMount } from 'svelte'
  
  /** @type {{ mac: string, ip: string }[]} */
  let clients = []
  let loading = true
  let errorMessage = ''

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

  /*
   Load the current client list when the dashboard opens.

   The WebSocket listens for access point client changes and immediately
   reloads the authoritative client list from the ESP32.
*/

  onMount(() => {
    loadClients()

    const protocol = 
      window.location.protocol === 'https:' ? 'wss:' : 'ws:'

    const socket = new WebSocket(
      `${protocol}//${window.location.host}/ws`
    )

    socket.onmessage = (event) => {
      try {
        const message = JSON.parse(event.data)

        if (message.event === 'clients_changed') {
          loadClients()
        }
      } catch {
        console.error('Invalid WebSocket message received')
      }
    }

    socket.onerror = () => {
      errorMessage = 'Dashboard live connection failed'
    }

    // Close the WebSocket when the dashboard is no longer active
    return () => {
      socket.close()
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
                <div class="flex items-center gap-2">
                  <span class="h-2 w-2 rounded-full bg-green-500"></span>

                  <span class="text-sm text-green-400">
                    Connected
                  </span>
                </div>
              </td>
            </tr>
          {/each}

          {#if !loading && clients.length === 0 && !errorMessage}
            <tr>
              <td
                colspan="3"
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
    </div>
  </section>
</main>