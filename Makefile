.PHONY: up down exec

up:
	UID=$$(id -u) GID=$$(id -g) docker compose up --build -d

down:
	docker compose down

exec:
	docker compose exec ros bash

up-gpu:
	UID=$$(id -u) GID=$$(id -g) docker compose \
		-f docker-compose.yml \
		-f docker-compose.gpu.yml \
		up --build -d

exec-gpu:
	docker compose \
		-f docker-compose.yml \
		-f docker-compose.gpu.yml \
		exec ros bash