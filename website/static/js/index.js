// Tactile Hands — vanilla JS for sticky-nav highlighting, BibTeX copy,
// and the scroll-to-top button. No jQuery, no Bulma carousel.

(function () {
  'use strict';

  // ---- BibTeX copy ----
  const copyBtn = document.querySelector('[data-copy]');
  const bibtex = document.getElementById('bibtex-code');
  if (copyBtn && bibtex) {
    copyBtn.addEventListener('click', async () => {
      const text = bibtex.textContent;
      const setCopied = () => {
        copyBtn.classList.add('is-copied');
        copyBtn.textContent = 'Copied';
        setTimeout(() => {
          copyBtn.classList.remove('is-copied');
          copyBtn.textContent = 'Copy';
        }, 1600);
      };
      try {
        await navigator.clipboard.writeText(text);
        setCopied();
      } catch {
        const ta = document.createElement('textarea');
        ta.value = text;
        document.body.appendChild(ta);
        ta.select();
        document.execCommand('copy');
        document.body.removeChild(ta);
        setCopied();
      }
    });
  }

  // ---- Scroll-to-top ----
  const scrollBtn = document.querySelector('[data-scroll-top]');
  if (scrollBtn) {
    const onScroll = () => {
      scrollBtn.classList.toggle('is-visible', window.scrollY > 320);
    };
    window.addEventListener('scroll', onScroll, { passive: true });
    onScroll();
    scrollBtn.addEventListener('click', () =>
      window.scrollTo({ top: 0, behavior: 'smooth' })
    );
  }

  // ---- Active section highlight in top nav ----
  const navLinks = Array.from(document.querySelectorAll('.topnav__links a'));
  const targets = navLinks
    .map((a) => document.querySelector(a.getAttribute('href')))
    .filter(Boolean);

  if ('IntersectionObserver' in window && targets.length) {
    const byId = new Map(
      navLinks.map((a) => [a.getAttribute('href').slice(1), a])
    );
    const io = new IntersectionObserver(
      (entries) => {
        entries.forEach((e) => {
          const link = byId.get(e.target.id);
          if (!link) return;
          if (e.isIntersecting) {
            navLinks.forEach((l) => l.classList.remove('is-active'));
            link.classList.add('is-active');
          }
        });
      },
      { rootMargin: '-40% 0px -55% 0px', threshold: 0 }
    );
    targets.forEach((t) => io.observe(t));
  }

  // ---- Theme toggle (light is the default) ----
  const themeButtons = Array.from(document.querySelectorAll('[data-theme-set]'));
  const themeMeta = { light: '#FAFAF7', dark: '#0E0E10' };

  function currentChoice() {
    try { return localStorage.getItem('theme') === 'dark' ? 'dark' : 'light'; }
    catch { return 'light'; }
  }

  function applyChoice(choice) {
    if (choice === 'dark') {
      document.documentElement.setAttribute('data-theme', 'dark');
      try { localStorage.setItem('theme', 'dark'); } catch {}
    } else {
      document.documentElement.removeAttribute('data-theme');
      try { localStorage.removeItem('theme'); } catch {}
    }
    themeButtons.forEach((b) => {
      b.setAttribute('aria-pressed', String(b.dataset.themeSet === choice));
    });
    document.querySelectorAll('meta[name="theme-color"]').forEach((m) => {
      m.setAttribute('content', themeMeta[choice]);
    });
  }

  if (themeButtons.length) {
    applyChoice(currentChoice());
    themeButtons.forEach((b) => {
      b.addEventListener('click', () => applyChoice(b.dataset.themeSet));
    });
  }

  // ---- Pause carousel videos when offscreen, play when in view ----
  const carouselVideos = document.querySelectorAll('.carousel__item video');
  if ('IntersectionObserver' in window && carouselVideos.length) {
    const vo = new IntersectionObserver(
      (entries) => {
        entries.forEach((e) => {
          const v = e.target;
          if (e.isIntersecting) v.play().catch(() => {});
          else v.pause();
        });
      },
      { threshold: 0.5 }
    );
    carouselVideos.forEach((v) => vo.observe(v));
  }
})();
