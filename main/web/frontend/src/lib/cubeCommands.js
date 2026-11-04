const CUBE_SIZE = 8

/**
 * @typedef {'clear' | 'show' | 'fill' | 'set_pixel'} CubeCommand
 */

// Define supported commands and their expected argument names
const COMMANDS = {
   clear: [],
   show: [],
   fill: ['r', 'g', 'b'],
   set_pixel: ['x', 'y', 'z', 'r', 'g', 'b']
}

/**
 * Parse comma-separated command arguments.
 * @param {string} text
 * @returns {string[]}
 */
function parseArguments(text)
{
   if (!text.trim()) return []

   // Split positional arguments without evaluatin user input
   return text.split(',').map(value => value.trim())
}

/**
 * Validate command arguments against their expected ranges.
 * @param {CubeCommand} command
 * @param {string[]} args
 * @returns {Record<string, number>}
 */
function validateArguments(command, args)
{
   const names = COMMANDS[command]

   if (args.length !== names.length) {
      throw new Error(
         `${command}() expects ${names.length} argument(s), received ${args.length}`
      )
   }

   /** @type {Record<string, number>} */ const params = {}

   for (let i = 0; i < names.length; i++) {
      const name = names[i]
      const value = args[i]

      // Only accept unsigned decimal integers
      if (!/^\d+$/.test(value)) {
         throw new Error(`${name} must be a non-negative integer`)
      }

      const number = Number(value)

      // Coordinates use cube dimensions; RGB channels use 8-bit values
      const max = ['x', 'y', 'z'].includes(name) ? CUBE_SIZE - 1 : 255

      if (!Number.isSafeInteger(number) || number > max) {
         throw new Error(`${name} must be between 0 and ${max}`)
      }
      params[name] = number
   }

   return params
}

/**
 * Parse a cube command.
 * @param {string} input
 */
export function parseCubeCommand(input)
{
   // Match the supported cumbe command syntax
   const match = input.trim().match(/^cube\.([a-z_]+)\((.*)\)$/)

   if (!match) {
      throw new Error('Invalid syntax. Expected cube.command(arguments)')
   }

   const [, commandName, argumentText] = match

   // Reject commands not explicitly registered.
   if (!Object.prototype.hasOwnProperty.call(COMMANDS, commandName)) {
      throw new Error(`Unknown command: ${commandName}`)
   }

   // The registry check ensures this is a supported command.
   /** @type {CubeCommand} */
   const command = /** @type {CubeCommand} */ (commandName)

   const args = parseArguments(argumentText)
   const params = validateArguments(command, args)

   // Return a transport independent command for the ESP32 dispatcher
   return { command, params }
}



