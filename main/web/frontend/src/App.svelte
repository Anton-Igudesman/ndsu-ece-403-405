<script>
  import Router from 'svelte-spa-router'

  import Home from './pages/Home.svelte'
  import AdminLogin from './pages/AdminLogin.svelte'
  import AdminDashboard from './pages/AdminDashboard.svelte'

  const routes = {
    '/': Home,
    '/admin': AdminLogin,
    '/admin/dashboard': AdminDashboard
  }
</script>

<Router {routes} />