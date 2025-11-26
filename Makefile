.PHONY: up down exec

up:
	UID=$$(id -u) GID=$$(id -g) docker compose up --build -d

down:
	docker compose down

exec:
	docker compose exec ros bash
