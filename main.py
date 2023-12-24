# tools
import logging
import argparse
# web application
from aiohttp import web
# handler
from handler.session import session_capture_handler, session_review_handler, on_shutdown
from handler.retrieve import query_scenes, remove_scene, query_info, define_base,  define_grid, define_landmark
# MVS
from SFM.core import MVS_PIPE

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="WebRTC-SLAM API Server")
  parser.add_argument("--host", default="0.0.0.0", help="Host for HTTP server (default: 0.0.0.0)")
  parser.add_argument("--port", type=int, default=8081, help="Port for HTTP server (default: 8081)")
  parser.add_argument("--cert", type=str, help="Path to the certificate file (for HTTPS)")
  parser.add_argument("--key", type=str, help="Path to the key file (for HTTPS)")
  parser.add_argument("--verbose", "-v", action="count")

  args = parser.parse_args()

  logging.basicConfig(level = (logging.DEBUG if args.verbose else logging.INFO))

  if args.cert:
      ssl_context = ssl.SSLContext()
      ssl_context.load_cert_chain(args.cert, args.key)
  else:
      ssl_context = None

  app = web.Application()
  app.on_shutdown.append(on_shutdown)
  app.router.add_post('/session/capture', session_capture_handler)
  app.router.add_post('/session/review', session_review_handler)
  app.router.add_get('/retrieve', query_scenes)
  app.router.add_post('/remove', remove_scene)
  app.router.add_get('/detail', query_info)
  app.router.add_post('/define', define_base)
  app.router.add_post('/gridding', define_grid)
  app.router.add_post('/landmark', define_landmark)
  app.router.add_static('/static', './static')

  MVS_PIPE.start()
  
  web.run_app(
    app, access_log=None, host=args.host, port=args.port, ssl_context=ssl_context,
  )