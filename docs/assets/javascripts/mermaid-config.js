(function () {
  'use strict';

  // ── Theme palettes ────────────────────────────────────────────────────────
  // Use 'default' / 'dark' as base so Mermaid handles text contrast internally;
  // we only override primary accent colours to match the Material indigo palette.
  var LIGHT = {
    theme: 'default',
    themeVariables: {
      primaryColor:       '#3f51b5',   // indigo-600
      primaryTextColor:   '#ffffff',
      primaryBorderColor: '#283593',   // indigo-900
      lineColor:          '#546e7a',
      fontSize:           '15px'
    }
  };

  var DARK = {
    theme: 'dark',
    themeVariables: {
      primaryColor:       '#5c7cfa',   // bright indigo
      primaryTextColor:   '#e8eaf6',
      primaryBorderColor: '#748ffc',
      lineColor:          '#90caf9',   // light blue arrows
      fontSize:           '15px'
    }
  };

  // ── Helpers ───────────────────────────────────────────────────────────────
  function isDark() {
    return document.body.getAttribute('data-md-color-scheme') === 'slate';
  }

  function buildCfg() {
    return Object.assign(
      { startOnLoad: false, securityLevel: 'antiscript' },
      isDark() ? DARK : LIGHT
    );
  }

  // Save raw diagram source before Mermaid replaces the div's content with SVG.
  function saveSources() {
    document.querySelectorAll('.mermaid:not([data-mz-src])').forEach(function (el) {
      el.setAttribute('data-mz-src', el.textContent.trim());
    });
  }

  // Restore raw source so the diagram can be re-rendered (e.g. after theme toggle).
  function restoreSources() {
    document.querySelectorAll('.mermaid[data-mz-src]').forEach(function (el) {
      el.textContent = el.getAttribute('data-mz-src');
      el.removeAttribute('data-processed');
    });
  }

  // ── Rendering ─────────────────────────────────────────────────────────────
  function render() {
    if (typeof mermaid === 'undefined') return;
    saveSources();
    mermaid.initialize(buildCfg());
    var result = mermaid.run({ querySelector: '.mermaid' });
    // mermaid.run() returns a Promise in v10+; add a polling fallback for safety.
    if (result && typeof result.then === 'function') {
      result.then(null, null); // attach nothing — zoom is handled by delegation below
    }
  }

  function rerender() {
    if (typeof mermaid === 'undefined') return;
    restoreSources();
    render();
  }

  // ── Zoom / lightbox — event delegation ───────────────────────────────────
  // A single listener on document catches every .mermaid click, regardless of
  // when diagrams finish rendering.  No per-element attachment needed.
  document.addEventListener('click', function (e) {
    // Ignore clicks that are already inside an open lightbox.
    if (e.target.closest('.mz-overlay')) return;

    var container = e.target.closest('.mermaid');
    if (!container) return;

    var svg = container.querySelector('svg');
    if (!svg) return;

    e.preventDefault();
    e.stopPropagation();
    openLightbox(svg);
  });

  function openLightbox(sourceSvg) {
    if (document.querySelector('.mz-overlay')) return;

    var overlay = document.createElement('div');
    overlay.className = 'mz-overlay';

    var clone = sourceSvg.cloneNode(true);
    clone.removeAttribute('data-mz-bound');
    clone.removeAttribute('style');
    clone.setAttribute('class', 'mz-svg');

    var btn = document.createElement('button');
    btn.className = 'mz-close';
    btn.setAttribute('aria-label', 'Close diagram');
    btn.innerHTML = '&times;';

    overlay.appendChild(clone);
    overlay.appendChild(btn);
    document.body.appendChild(overlay);
    addPanZoom(overlay, clone);

    function close() {
      if (overlay.parentNode) overlay.parentNode.removeChild(overlay);
      document.removeEventListener('keydown', onKey);
    }
    function onKey(e) { if (e.key === 'Escape') close(); }

    overlay.addEventListener('click', close);
    clone.addEventListener('click', function (e) { e.stopPropagation(); });
    btn.addEventListener('click', function (e) { e.stopPropagation(); close(); });
    document.addEventListener('keydown', onKey);
  }

  function addPanZoom(overlay, svg) {
    var scale = 1, tx = 0, ty = 0;
    var dragging = false, ox = 0, oy = 0;

    svg.style.transformOrigin = 'center center';
    svg.style.transition = 'transform 0.08s ease-out';
    svg.style.cursor = 'grab';

    function apply() {
      svg.style.transform =
        'translate(' + tx + 'px, ' + ty + 'px) scale(' + scale + ')';
    }

    svg.addEventListener('mousedown', function (e) {
      e.stopPropagation();
      dragging = true;
      ox = e.clientX - tx;
      oy = e.clientY - ty;
      svg.style.cursor = 'grabbing';
      svg.style.transition = 'none';
    });
    window.addEventListener('mousemove', function (e) {
      if (!dragging) return;
      tx = e.clientX - ox;
      ty = e.clientY - oy;
      apply();
    });
    window.addEventListener('mouseup', function () {
      if (!dragging) return;
      dragging = false;
      if (svg.parentNode) svg.style.cursor = 'grab';
    });
    overlay.addEventListener('wheel', function (e) {
      e.preventDefault();
      scale = Math.min(10, Math.max(0.2, scale + (e.deltaY < 0 ? 0.15 : -0.15)));
      apply();
    }, { passive: false });
  }

  // ── Boot ──────────────────────────────────────────────────────────────────
  render();

  // Re-render after Material instant-navigation page swaps.
  if (typeof document$ !== 'undefined') {
    document$.subscribe(function () {
      setTimeout(render, 0);
    });
  }

  // Re-render when user toggles light ↔ dark mode.
  new MutationObserver(function (mutations) {
    for (var i = 0; i < mutations.length; i++) {
      if (mutations[i].attributeName === 'data-md-color-scheme') {
        rerender();
        return;
      }
    }
  }).observe(document.body, {
    attributes: true,
    attributeFilter: ['data-md-color-scheme']
  });

})();
