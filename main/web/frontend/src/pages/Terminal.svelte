
<script>
   import { parseCubeCommand } from '../lib/cubeCommands.js'

   /** @type {string} */
   let command = ''

   /** @type {{ type: string, text: string }[]} */
   let history = [
      { type: 'system', text: 'LED Cube Terminal v0.1' },
      { type: 'system', text: 'Type help() for available commands.' }
   ]

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
    * Validate and display a parsed cube command.
    * @param {string} input
    * @returns {void}
    */
   function processCubeCommand(input)
   {
      try {
         // Parse and validate without sending commands to the ESP32.
         const result = parseCubeCommand(input)

         addHistory('output', JSON.stringify(result, null, 3))
      } catch (error) {
         // Report invalid commands without interrupting the terminal.
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
</style>
