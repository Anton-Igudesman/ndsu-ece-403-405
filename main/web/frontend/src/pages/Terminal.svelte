
<script>
   import { onMount } from 'svelte'
   import { parseCubeCommand } from '../lib/cubeCommands.js'

   /** @type {string} */
   let command = ''

   /** @type {{ type: string, text: string }[]} */
   let history = [
      { type: 'system', text: 'LED Cube Terminal v0.1' },
      { type: 'system', text: 'Type help() for available commands.' }
   ]

   /** @type {WebSocket | null} */
   let terminalSocket = null

   /** @type {boolean} */
   let connected = false

   /** @type {boolean} */
   let isTerminalOwner = false

   /** @type {boolean} */
   let terminalOwnerPresent = false

   /** @type {number[]} */
   let pendingRequests = []

   /** @type {boolean} */
   let isRequestPending = false

   /** @type {boolean} */
   let commandModeActive = false

   /** @type {ReturnType<typeof setInterval> | null} */
   let modeStatusTimer = null
   
   /**
   * Stop polling the ESP32 for Command Mode status.
   * @returns {void}
   */
   function stopModeStatusPolling()
   {
      if (modeStatusTimer === null) return

      clearInterval(modeStatusTimer)
      modeStatusTimer = null
   }

   /**
   * Poll until the LED controller finishes entering Command Mode.
   * @returns {void}
   */
   function startModeStatusPolling()
   {
      stopModeStatusPolling()

      // Mode transitions complete asynchronously in the LED task.
      requestModeStatus()

      modeStatusTimer = setInterval(() => {
         if (!connected) {
            stopModeStatusPolling()
            return
         }

         requestModeStatus()
      }, 250)
   }

   /**
    * Request Command Mode activation.
    * @returns {void}
    */
   function activateCommandMode()
   {
      if (!connected || terminalSocket?.readyState !== WebSocket.OPEN) {
         addHistory('error', 'LED cube is not connected')
         return
      }

      // Request a mode change through the existing terminal connection
      terminalSocket.send(JSON.stringify({
         action: 'set_mode',
         mode: 'command'
      }))
   }

   /**
   * Request the current Command Mode status from the ESP32.
   * @returns {void}
   */
   function requestModeStatus()
   {
      if (terminalSocket?.readyState !== WebSocket.OPEN) return

      terminalSocket.send(JSON.stringify({
         action: 'get_mode'
      }))
   }

   /**
   * Request the current terminal ownership status.
   * @returns {void}
   */
   function requestOwnershipStatus()
   {
      if (terminalSocket?.readyState !== WebSocket.OPEN) return

      terminalSocket.send(JSON.stringify({
         action: 'get_ownership'
      }))
   }

   /**
   * Request terminal control from the current owner.
   * @returns {void}
   */
   function requestTerminalControl()
   {
      if (terminalSocket?.readyState !== WebSocket.OPEN) return

      terminalSocket.send(JSON.stringify({
         action: 'request_control'
      }))
   }

   /**
   * Voluntarily release terminal control.
   * @returns {void}
   */
   function releaseTerminalControl()
   {
      if (terminalSocket?.readyState !== WebSocket.OPEN) return
      if (!isTerminalOwner) return

      terminalSocket.send(JSON.stringify({
         action: 'release_control'
      }))
   }

   /**
   * Grant terminal control to a requesting device.
   * @param {number} requesterFd
   * @returns {void}
   */
   function grantTerminalControl(requesterFd)
   {
      if (terminalSocket?.readyState !== WebSocket.OPEN) return
      if (!isTerminalOwner) return

      terminalSocket.send(JSON.stringify({
         action: 'grant_control',
         requester_fd: requesterFd
      }))
   }

   /**
   * Deny a pending terminal control request.
   * @param {number} requesterFd
   * @returns {void}
   */
   function denyTerminalControl(requesterFd)
   {
      if (terminalSocket?.readyState !== WebSocket.OPEN) return
      if (!isTerminalOwner) return

      terminalSocket.send(JSON.stringify({
         action: 'deny_control',
         requester_fd: requesterFd
      }))
   }

   /**
   * Take terminal ownership as an administrator.
   * @returns {void}
   */
   function adminTakeoverTerminal()
   {
      if (terminalSocket?.readyState !== WebSocket.OPEN) return

      terminalSocket.send(JSON.stringify({
         action: 'admin_takeover'
      }))
   }

   /**
   * Update the pending terminal control requests.
   * @param {number[]} requests
   * @returns {void}
   */
   function updatePendingRequests(requests)
   {
      // Keep only valid, nonnegative socket descriptors
      pendingRequests = requests.filter(
         (requesterFd) => Number.isInteger(requesterFd) && requesterFd >= 0
      )
   }

   /**
    * Connect to the ESP32 terminal WebSocket.
    * @returns {void}
    */
   function connectTerminal()
   {
      const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
      const url = `${protocol}//${window.location.host}/ws/terminal`

      terminalSocket = new WebSocket(url)

      terminalSocket.onopen = () => {
         connected = true
         addHistory('system', 'Connected to LED cube.')

         // Retrieve the actual mode
         requestModeStatus()

         // Retrieve terminal ownership when the WebSocket connects
         requestOwnershipStatus()
      }

      terminalSocket.onmessage = (event) => {
         try {
            const response = JSON.parse(String(event.data))

            // Update terminal ownership when the server reports a change
            if (response.event === 'terminal_ownership_changed') {
               isTerminalOwner = response.is_owner === true
               terminalOwnerPresent = response.owner_present === true
               isRequestPending = response.request_pending === true
               return
            }

            // Report whether the terminal control request succeeded
            if (response.event === 'control_request_result') {
               if (!response.success)
                  addHistory('error', response.error || 'Unable to request terminal control')
               return
            }

            // Report the result of terminal ownership actions
            if ([
               'control_release_result',
               'control_grant_result',
               'control_deny_result',
               'control_takeover_result'
            ].includes(response.event)) {
               if (!response.success)
                  addHistory('error', response.error || 'Terminal control action failed')

               return
            }

            // Update pending control requests received from the server
            if (response.event === 'terminal_pending_requests') {
               const requests = Array.isArray(response.requests)
                  ? response.requests.map(
                     /** @param {{ requester_fd: number }} request */
                     (request) => request.requester_fd)
                  : []

               updatePendingRequests(requests)
               return
            }

            if (response.event === 'mode_status') {
               commandModeActive = response.success === true
               
               // Stop polling once Command Mode becomes active
               if (commandModeActive) stopModeStatusPolling()

               return
            }

            if (response.event === 'mode_activation_result') {
               if (response.success) {
                  // Wait for LED controller to finish transition
                  startModeStatusPolling()
               } else {
                  stopModeStatusPolling()
                  addHistory('error', response.error || 'Mode activation failed')
               }
               return
            }

            if (response.event === 'terminal_result') {
               addHistory(
                  response.success ? 'output' : 'error',
                  response.success ? 'OK' : (response.error || 'Command failed')
               )
            }
         } catch {
            addHistory('error', 'Invalid response from LED cube.')
         }
      }

      terminalSocket.onclose = () => {
         stopModeStatusPolling()
         commandModeActive = false

         // Clear ownership state when the terminal connection closes
         isTerminalOwner = false
         terminalOwnerPresent = false
         isRequestPending = false

         // Clear pending requests when the terminal connection closes
         pendingRequests = []

         connected = false
         terminalSocket = null
         addHistory('error', 'Disconnected from LED cube.')
      }

      terminalSocket.onerror = () => {
         addHistory('error', 'Terminal connection error.')
      }
   }

   onMount(() => {
      connectTerminal()

      return () => {
         // Close the connection when leaving the terminal page.
         stopModeStatusPolling()
         terminalSocket?.close()
      }
   })

   /**
    * Display a message in the terminal history.
    * @param {string} type
    * @param {string} text
    * @returns {void}
    */
   function addHistory(type, text)
   {
      history = [...history, { type, text }]
   }

   /**
    * Display supported terminal commands.
    * @returns {void}
    */
   function showHelp()
   {
      addHistory('output', [
         'Available commands:',
         '  clear()                  Clear terminal screen',
         '  cube.clear()',
         '  cube.fill(r, g, b)',
         '  cube.set_pixel(x, y, z, r, g, b)',
         '  cube.show()'
      ].join('\n'))
   }

   /**
    * Validate and send a cube command to ESP32
    * @param {string} input
    * @returns {void}
    */
   function processCubeCommand(input)
   {
      try {
         // Validate before sending commands to the ESP32.
         const result = parseCubeCommand(input)

         // Keep hardware commands inactive until Command Mode is confirmed
         if (!commandModeActive) {
            addHistory('error', 'Activate Command Mode before sending commands')
            return
         }

         if (!connected || terminalSocket?.readyState !== WebSocket.OPEN) {
            addHistory('error', 'LED cube is not connected')
            return
         }

         terminalSocket.send(JSON.stringify(result))
      } catch (error) {
         const message = error instanceof Error ? error.message : String(error)
         addHistory('error', message)
      }
   }

   /**
   * Clear the terminal output without affecting the LED cube.
   * @returns {void}
   */
   function clearTerminal()
   {
      history = []
   }

   /**
   * Process the current terminal input.
   * @returns {void}
   */
   function executeCommand()
   {
      const input = command.trim()

      if (!input) return

      // Clear the terminal without preserving the command in history.
      if (input === 'clear()') {
         clearTerminal()
         command = ''
         return
      }

      addHistory('input', `>>> ${input}`)

      if (input === 'help()' || input === 'cube.help()') {
         showHelp()
      } else {
         processCubeCommand(input)
      }

      command = ''
   }
</script>

<div class="terminal-page">
   <h1>LED Cube Terminal</h1>
   <p>Interactive command interface</p>

   {#if connected && !isTerminalOwner}
      <button
         type="button"
         disabled={isRequestPending}
         on:click={requestTerminalControl}
      >
         {isRequestPending ? 'Request Pending' : 'Request Control'}
      </button>
   {/if}
   <div class="mode-panel">
      <div class="mode-info">
         <span class="mode-indicator" class:active={commandModeActive}></span>

         <div>
            <strong>
               Command Mode {commandModeActive ? 'Active' : 'Inactive'}
            </strong>
            <p>
               {commandModeActive
                  ? 'The LED cube is ready to receive commands.'
                  : 'Activate Command Mode to control the LED cube.'}
            </p>
         </div>
      </div>

      {#if !commandModeActive}
         <button
            type="button"
            class="activate-button"
            disabled={!connected}
            on:click={activateCommandMode}
         >
            Activate Command Mode
         </button>
      {/if}
   </div>

   <div class="terminal">
      <div class="terminal-output">
         {#each history as entry}
            <div class={entry.type}>{entry.text}</div>
         {/each}
      </div>

      <form on:submit|preventDefault={executeCommand}>
         <span class="prompt">&gt;&gt;&gt;</span>
         <input
            type="text"
            bind:value={command}
            autocomplete="off"
            spellcheck="false"
            placeholder="Enter a cube command..."
         />
      </form>
   </div>
</div>

<style>
   .terminal-page {
      padding: 24px;
      max-width: 1000px;
      margin: auto;
   }

   .terminal {
   display: flex;
   flex-direction: column;
   height: 450px;
   box-sizing: border-box;
   background: #101827;
   color: #e5e7eb;
   border-radius: 8px;
   padding: 20px;
   font-family: monospace;
}

   .terminal-output {
      flex: 1;
      min-height: 0;
      overflow-y: auto;
      overflow-x: hidden;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
      margin-bottom: 16px;
   }

   .terminal-output > div {
      margin-bottom: 6px;
   }

   .system {
      color: #93c5fd;
   }

   .input {
      color: #f9fafb;
   }

   .output {
      color: #86efac;
   }

   .error {
      color: #f87171;
   }

   form {
      display: flex;
      flex-shrink: 0;
      gap: 10px;
      align-items: center;
   }

   .prompt {
      color: #86efac;
   }

   input {
      flex: 1;
      background: transparent;
      border: none;
      outline: none;
      color: white;
      font-family: monospace;
      font-size: 14px;
   }

   
.mode-panel {
   display: flex;
   justify-content: space-between;
   align-items: center;
   gap: 20px;
   padding: 18px 20px;
   margin-bottom: 20px;
   border: 1px solid #334155;
   border-radius: 8px;
   background: #1e293b;
   color: #f8fafc;
}

.mode-info {
   display: flex;
   align-items: center;
   gap: 12px;
}

.mode-info p {
   margin: 5px 0 0;
   color: #94a3b8;
   font-size: 13px;
}

.mode-indicator {
   width: 10px;
   height: 10px;
   flex-shrink: 0;
   border-radius: 50%;
   background: #ef4444;
}

.mode-indicator.active {
   background: #22c55e;
}

.activate-button {
   padding: 10px 16px;
   border: none;
   border-radius: 6px;
   background: #2563eb;
   color: white;
   font-weight: 600;
   cursor: pointer;
}

.activate-button:hover:not(:disabled) {
   background: #1d4ed8;
}

.activate-button:disabled {
   opacity: 0.5;
   cursor: not-allowed;
}

</style>
