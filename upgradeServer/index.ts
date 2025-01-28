import Bun from "bun";

const PROJNAME = "HK-Garage";

const upgradeFile = Bun.file(`${import.meta.dir}/../build/${PROJNAME}.bin`);

if (!(await upgradeFile.exists())) {
  console.error("Project not compiled!");
} else {
  Bun.serve({
    port: 80,
    async fetch(req, server) {
      const url = new URL(req.url);

      if (url.pathname === "/upgrade") {
        return new Response(upgradeFile);
      }
      if (url.pathname === "/probe") return new Response("Server is up!");
      return new Response("404!");
    }
  });

  console.log("Server started!");
}
