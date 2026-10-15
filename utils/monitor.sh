#!/bin/bash
set -e

REMOTE_USER="root"
REMOTE_HOST="roboclean.local"

echo "Tailing live logs from $REMOTE_HOST..."
ssh -t "$REMOTE_USER@$REMOTE_HOST" "docker compose -f docker-compose.prod.yml logs -f"
