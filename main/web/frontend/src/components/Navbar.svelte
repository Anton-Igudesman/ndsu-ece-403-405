<script>
  import { onMount } from 'svelte';
  import { link, router } from 'svelte-spa-router';

  const navigation = [
    { label: 'Home', path: '/' },
    { label: 'Terminal', path: '/terminal' },
    { label: 'Effects', path: '/effects' }
  ];

  // Track administrator authorization for dashboard navigation
  let isAdmin = $state(false);

  // Retrieve the current device's authorization state
  async function refreshAdminStatus() {
    try {
      const response = await fetch('/api/controller/status', {
        cache: 'no-store'
      });

      if (!response.ok) {
        isAdmin = false;
        return;
      }

      const data = await response.json();

      // CLIENT_AUTH_ADMIN is defined as 3 in the firmware
      isAdmin = data.auth_state === 3;
    } catch (error) {
      console.error('Unable to retrieve administrator status:', error);
      isAdmin = false;
    }
  }

  onMount(() => {
    refreshAdminStatus();

    // Refresh authorization when navigating between pages
    const handleNavigation = () => {
      refreshAdminStatus();
    };

    window.addEventListener('hashchange', handleNavigation);

    return () => {
      window.removeEventListener('hashchange', handleNavigation);
    };
  });
</script>

<nav class="w-full border-b border-slate-800 bg-slate-950">
  <div class="mx-auto flex max-w-5xl flex-wrap items-center justify-center gap-3 px-4 py-4">
    {#each navigation as item}
      <a
        href={item.path}
        use:link
        class={`rounded-lg px-4 py-2 text-sm font-semibold transition-colors ${
          router.location === item.path
            ? 'bg-blue-600 text-white'
            : 'bg-slate-800 text-slate-300 hover:bg-slate-700'
        }`}
      >
        {item.label}
      </a>
    {/each}
    <!-- Show Admin Dashboard or Admin Login based on authorization -->
    <a
      href={isAdmin ? '/admin/dashboard' : '/admin'}
      use:link
      class={`rounded-lg px-4 py-2 text-sm font-semibold transition-colors ${
        router.location === (isAdmin ? '/admin/dashboard' : '/admin')
          ? 'bg-blue-600 text-white'
          : 'bg-slate-800 text-slate-300 hover:bg-slate-700'
      }`}
    >
      {isAdmin ? 'Admin Dashboard' : 'Admin Login'}
    </a>
  </div>
</nav>