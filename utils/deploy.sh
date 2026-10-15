#!/bin/bash
set -e

REMOTE_USER="root"
REMOTE_HOST="roboclean.local"
IMAGE_NAME="roboclean:prod"
TAR_FILE="roboclean.tar"

# Ensure the project root
cd "$(dirname "$0")/.."

echo "[1/6] Building production image for ARM64..."
docker buildx build --platform linux/arm64 -f Dockerfile.prod -t "$IMAGE_NAME" --output type=docker,dest="./$TAR_FILE" .

echo "[2/6] Stopping containers on remote host..."
ssh "$REMOTE_USER@$REMOTE_HOST" "docker compose -f docker-compose.prod.yml down || true"

echo "[3/6] Transferring image to remote host..."
scp "$TAR_FILE" "$REMOTE_USER@$REMOTE_HOST:~/"

echo "[4/6] Loading image on remote host..."
ssh "$REMOTE_USER@$REMOTE_HOST" "docker load -i $TAR_FILE"

echo "[5/6] Starting containers on remote host..."
ssh "$REMOTE_USER@$REMOTE_HOST" "docker compose -f docker-compose.prod.yml up -d --force-recreate"

echo "[6/6] New image deployed successfully."
